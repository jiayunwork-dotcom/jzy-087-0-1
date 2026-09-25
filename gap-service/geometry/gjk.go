package geometry

import "math"

// 接触法向约定：一律取为「把 B 从 A 上推开」的单位方向。
// 分离时 n 由 A 上的最近点指向 B 上的最近点；穿透时 n 是差集最近边的外法向。

// Status 是核算结论。
type Status string

const (
	StatusSeparated   Status = "separated"
	StatusPenetrating Status = "penetrating"
)

// Result 是间隙 / 穿透核算结果。
//
// 分离分支：Distance>0 为最近距离，Normal 为最近点连线方向，
// PointOnA/PointOnB 为最近点对；
// 穿透分支：Depth 为穿透深度，Normal 为把 B 相对 A 恰好推开的方向，
// PointOnA/PointOnB 为接触点（用于装配参考，穿透状态下二者并非唯一）。
type Result struct {
	Status   Status
	Distance float64 // 仅分离分支有效
	Depth    float64 // 仅穿透分支有效
	Normal   Point   // 单位向量：推开 B 的方向
	PointOnA Point
	PointOnB Point
	Steps    int // GJK 单纯形迭代次数
}

// gjkOutput 是 GJK 单纯形迭代的内部产物。
type gjkOutput struct {
	hit     bool             // 原点是否在差集内（含恰好在边界上的接触）
	simp    []MinkowskiPoint // 最终单纯形
	closest closest          // 分离时：原点到单纯形的最近特征
	steps   int
	touch   bool // 原点恰好落在差集边界上（距离为 0 的接触）
}

// Query 对两个凸多边形执行 GJK/EPA 核算。入参会先做凸性与退化校验，
// 凹多边形直接拒绝（不做凸分解）。
func Query(a, b []Point) (Result, error) {
	if err := ValidatePolygon(a); err != nil {
		return Result{}, err
	}
	if err := ValidatePolygon(b); err != nil {
		return Result{}, err
	}

	out, err := runGJK(a, b)
	if err != nil {
		return Result{}, err
	}

	if !out.hit {
		return separatedResult(out), nil
	}
	if out.touch {
		// 零距离接触：交给 EPA 构造退化多面体，给出确定的接触法向，
		// 深度为 0，结论仍是穿透（两体未分离），不允许报成“距离 0 的分离”。
		return epa(a, b, out.simp, out.steps, true)
	}
	return epa(a, b, out.simp, out.steps, false)
}

// runGJK 在 Minkowski 差 A⊖B 上做单纯形迭代（距离版 GJK）：
// 每轮把差集沿“当前最近点 -> 原点”方向的支撑点加入单纯形，再演化单纯形。
// 新支撑点相对过最近点的垂直平面不再前进（差距容差内）即收敛为分离；
// 单纯形包住原点即穿透。
func runGJK(a, b []Point) (gjkOutput, error) {
	// 初始方向：由 A 上的一个参考点（首顶点）指向 B 的首顶点，
	// 保证差集上首个支撑点的选取确定。
	d := b[0].sub(a[0])
	if d.len2() <= EpsAbs*EpsAbs {
		d = Point{X: 1}
	}
	simp := []MinkowskiPoint{supportMinkowski(a, b, d)}

	var c closest
	var kept []MinkowskiPoint
	for step := 1; step <= MaxGJKSteps; step++ {
		// 先在当前单纯形上求最近点并演化子集。
		var inside bool
		c, kept, _, inside = nearestOnSimplex(simp)
		if inside {
			return gjkOutput{hit: true, simp: simp, steps: step}, nil
		}

		dist := c.point.length()
		scale := math.Max(1.0, c.point.length())
		linTol := EpsAbs * scale

		// 原点落在单纯形上：零距离接触。
		if dist <= linTol {
			return gjkOutput{hit: true, simp: kept, steps: step, touch: true}, nil
		}

		// 沿最近点方向（c.point -> 原点）取差集支撑。
		d = c.d
		wp := supportMinkowski(a, b, d)

		// 与现有单纯形顶点重复：差集在该方向上无法再前进。
		if wp.V.sub(simp[0].V).len2() <= linTol*linTol ||
			(len(simp) > 1 && wp.V.sub(simp[1].V).len2() <= linTol*linTol) ||
			(len(simp) > 2 && wp.V.sub(simp[2].V).len2() <= linTol*linTol) {
			return gjkOutput{hit: false, simp: kept, closest: c, steps: step}, nil
		}

		// 收敛判据（距离版 GJK）：设 d_hat 为当前最近点指向原点的单位向量，
		// 过最近点且垂直于 d_hat 的支撑平面到原点的距离为 dist。
		// 新支撑点 w 在 d_hat 方向上把该平面推进的量为 w·d_hat + dist
		// （最近点处 w·d_hat = -dist，该值为 0）。推进量落入容差即收敛，
		// dist 就是原点到差集的真实最近距离。
		gap := wp.V.dot(d.normalized()) + dist
		if gap <= linTol {
			return gjkOutput{hit: false, simp: kept, closest: c, steps: step}, nil
		}

		// 加入新支撑点，保持单纯形顶点顺序（旧子集在前、新点在后），
		// 三角形分支不依赖环绕方向，最近边枚举覆盖三种组合。
		simp = append(kept, wp)
	}
	return gjkOutput{}, ErrGJKNotConverged
}

// separatedResult 由收敛的最近特征组装分离结论。
//
// 差集上的最近点 w = pA - pB 由 B 上的最近点指向 A 上的最近点，
// 故「把 B 从 A 推开」的接触法向取 -w 的单位化方向，即由 pA 指向 pB。
func separatedResult(out gjkOutput) Result {
	c := out.closest
	n := c.point.neg().normalized()
	if n.len2() == 0 {
		n = Point{X: 1}
	}
	return Result{
		Status:   StatusSeparated,
		Distance: c.point.length(),
		Normal:   n,
		PointOnA: c.a,
		PointOnB: c.b,
		Steps:    out.steps,
	}
}
