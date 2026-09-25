package geometry

import (
	"errors"
	"math"
)

// epaEdge 是 EPA 多面体（平面上为凸多边形）的一条边。
type epaEdge struct {
	i, j int     // hull 中首尾顶点下标（hull 保持逆时针环绕）
	n    Point   // 边的外法向（单位向量）
	dist float64 // 原点到边所在直线的有符号距离（外法向侧为正）
}

// epa 执行多面体扩展（Expanding Polytope Algorithm）：
// 从包住原点的初始单纯形出发，反复取距原点最近的边，沿其外法向求差集的
// 支撑点并把它插入多面体；当支撑点不再越过多面体最近边时，该边的距离即
// 穿透深度，外法向即把 B 从 A 上推开的方向。
func epa(a, b []Point, seed []MinkowskiPoint, gjkSteps int, touch bool) (Result, error) {
	hull, err := buildSeedHull(a, b, seed, touch)
	if err != nil {
		var se *seedError
		if errors.As(err, &se) {
			return zeroDepthContact(seed, gjkSteps), nil
		}
		return Result{}, err
	}

	for step := 1; step <= MaxEPASteps; step++ {
		edge := nearestEdge(hull)

		p := supportMinkowski(a, b, edge.n)
		scale := math.Max(1.0, math.Max(math.Abs(p.V.X), math.Abs(p.V.Y)))
		linTol := EpsAbs * scale

		// 支撑点到最近边所在直线的（外法向方向）距离。
		// 若与边距没有显著差别，多面体边界已贴合差集边界，收敛。
		d := p.V.dot(edge.n)
		if d-edge.dist <= linTol {
			pa, pb := edgeContact(hull, edge)
			depth := edge.dist
			if depth < 0 {
				depth = 0
			}
			return Result{
				Status:   StatusPenetrating,
				Depth:    depth,
				Normal:   edge.n,
				PointOnA: pa,
				PointOnB: pb,
				Steps:    gjkSteps + step,
			}, nil
		}

		hull, err = expandHull(hull, edge.i, edge.j, p)
		if err != nil {
			return Result{}, err
		}
	}
	return Result{}, ErrEPANotConverged
}

// nearestEdge 枚举逆时针凸多边形的各边，返回原点到其直线距离最小的边。
// 原点在多边形内部，左侧法向（leftNormal）即外法向，有符号距离非负。
func nearestEdge(hull []MinkowskiPoint) epaEdge {
	best := epaEdge{dist: math.Inf(1)}
	n := len(hull)
	for i := 0; i < n; i++ {
		j := (i + 1) % n
		nn := leftNormal(hull[i].V, hull[j].V).normalized()
		d := hull[i].V.dot(nn)
		if d < best.dist {
			best = epaEdge{i: i, j: j, n: nn, dist: d}
		}
	}
	return best
}

// edgeContact 用最近边两端点在原点投影处的插值系数，把接触点还原到
// A、B 两个多边形上。
func edgeContact(hull []MinkowskiPoint, e epaEdge) (Point, Point) {
	p, q := hull[e.i].V, hull[e.j].V
	d := q.sub(p)
	// 原点在边直线上的投影：p + t*(q-p)，满足 (p+t d)·n = 0 形式；
	// 这里最近点一般在线段内部，t = -p·d / |d|²。
	t := -p.dot(d) / d.len2()
	t = math.Max(0, math.Min(1, t))
	pa := hull[e.i].A.scale(1 - t).add(hull[e.j].A.scale(t))
	pb := hull[e.i].B.scale(1 - t).add(hull[e.j].B.scale(t))
	return pa, pb
}

// expandHull 把支撑点 p 插入逆时针凸包：
// 删除从最近边开始、对 p 可见的连续边（p 在边的外法向一侧），
// 用两条新边把 p 接到可见链两端。必要时旋转列表以处理环绕点 0 的情况。
func expandHull(hull []MinkowskiPoint, ei, ej int, p MinkowskiPoint) ([]MinkowskiPoint, error) {
	n := len(hull)

	// 从最近边 (ei,ej) 向两侧扩展可见链 [start, end]（含两端顶点）。
	start, end := ei, ej
	visible := func(i, j int) bool {
		nn := leftNormal(hull[i].V, hull[j].V).normalized()
		return p.V.dot(nn) > hull[i].V.dot(nn)+EpsAbs*math.Max(1, hull[i].V.length())
	}

	// 向前扩展 start。
	for {
		ps := (start - 1 + n) % n
		if visible(ps, start) {
			start = ps
			if start == end {
				return nil, ErrEPANotConverged
			}
			continue
		}
		break
	}
	// 向后扩展 end。
	for {
		ne := (end + 1) % n
		if visible(end, ne) {
			end = ne
			if end == start {
				return nil, ErrEPANotConverged
			}
			continue
		}
		break
	}

	// 旋转使 start 位于列表开头，便于直接拼接。
	rot := make([]MinkowskiPoint, 0, n+1)
	rot = append(rot, hull[start:]...)
	rot = append(rot, hull[:start]...)
	// 旋转后 end 的新下标（保持原循环顺序）。
	endIdx := (end - start + n) % n

	out := make([]MinkowskiPoint, 0, n-endIdx+2)
	out = append(out, rot[0])          // start
	out = append(out, p)               // 新支撑点
	out = append(out, rot[endIdx:]...) // end 及其后保持不变的顶点
	return out, nil
}

// zeroDepthContact 处理构造不出包围种子的零距离接触：
// 用退化为线段/点的单纯形给出确定的接触法向（边的外法向，缺省取 +x），
// 深度为 0，结论仍为穿透（两体没有可分离的间隙）。
func zeroDepthContact(seed []MinkowskiPoint, steps int) Result {
	var n Point
	var pa, pb Point
	switch {
	case len(seed) >= 2:
		n = leftNormal(seed[0].V, seed[1].V).normalized()
		pa = seed[0].A.scale(0.5).add(seed[1].A.scale(0.5))
		pb = seed[0].B.scale(0.5).add(seed[1].B.scale(0.5))
	case len(seed) == 1:
		pa, pb = seed[0].A, seed[0].B
	}
	if n.len2() == 0 {
		n = Point{X: 1}
	}
	return Result{
		Status:   StatusPenetrating,
		Depth:    0,
		Normal:   n,
		PointOnA: pa,
		PointOnB: pb,
		Steps:    steps,
	}
}
