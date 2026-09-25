package geometry

import "math"

// closest 描述当前单纯形到原点的最近特征。
type closest struct {
	point Point // 单纯形上离原点最近的点
	a, b  Point // 用单纯形顶点的重心系数还原出的 A、B 上的点
	d     Point // 指向原点的下一个搜索方向（point 的反向）
}

// nearestOnSimplex 求原点到当前单纯形的最近点，并执行单纯形演化：
// 返回的 kept 是保留下来的子集（1、2 个顶点的单纯形），lambda 是 kept 上
// 对应重心系数，inside 表示原点是否落在三顶点单纯形内部（穿透分支）。
func nearestOnSimplex(simp []MinkowskiPoint) (c closest, kept []MinkowskiPoint, lambda []float64, inside bool) {
	switch len(simp) {
	case 1:
		c.point = simp[0].V
		c.a, c.b = simp[0].A, simp[0].B
		c.d = c.point.neg()
		return c, simp, []float64{1}, false

	case 2:
		p0, p1 := simp[0].V, simp[1].V
		ab := p1.sub(p0)
		// t = clamp((O-a)·(b-a) / |b-a|²)：最近点取在线段上的垂足。
		t := p0.neg().dot(ab) / ab.len2()
		if t < 0 {
			t = 0
		} else if t > 1 {
			t = 1
		}
		lambda = []float64{1 - t, t}
		c.point = p0.add(ab.scale(t))
		c.a = simp[0].A.scale(lambda[0]).add(simp[1].A.scale(lambda[1]))
		c.b = simp[0].B.scale(lambda[0]).add(simp[1].B.scale(lambda[1]))
		c.d = c.point.neg()

		if t <= EpsAbs {
			return c, simp[:1], []float64{1}, false
		}
		if t >= 1-EpsAbs {
			return c, simp[1:], []float64{1}, false
		}
		// 保留边，顺序取 (a,b)。
		return c, []MinkowskiPoint{simp[0], simp[1]}, lambda, false

	default: // 3 个顶点：三角形
		a, b, cc := simp[0].V, simp[1].V, simp[2].V
		if originInTriangle(a, b, cc) {
			return closest{}, nil, nil, true
		}
		// 原点在三角形外：单纯形退化为离原点最近的那条边，
		// 按边 (p_i, p_j) 的顺序保留，交给下一轮迭代。
		edges := [3][2]int{{0, 1}, {1, 2}, {2, 0}}
		bestI := 0
		var bestDist2 float64 = math.Inf(1)
		var bestT float64
		for i, e := range edges {
			p, q := simp[e[0]].V, simp[e[1]].V
			d := q.sub(p)
			t := p.neg().dot(d) / d.len2()
			t = math.Max(0, math.Min(1, t))
			proj := p.add(d.scale(t))
			if d2 := proj.len2(); d2 < bestDist2 {
				bestDist2 = d2
				bestI = i
				bestT = t
			}
		}
		e := edges[bestI]
		p, q := simp[e[0]], simp[e[1]]
		lambda = []float64{1 - bestT, bestT}
		c.point = p.V.add(q.V.sub(p.V).scale(bestT))
		c.a = p.A.scale(lambda[0]).add(q.A.scale(lambda[1]))
		c.b = p.B.scale(lambda[0]).add(q.B.scale(lambda[1]))
		c.d = c.point.neg()
		return c, []MinkowskiPoint{p, q}, lambda, false
	}
}

// originInTriangle 判定原点是否落在三角形 (a,b,c) 内部或边界上。
// 要求 a,b,c 逆时针（或顺时针）环绕；三边叉积同号即在内部。
// 判据使用与三角形尺度相关的容差，并把共线退化（面积≈0）视为“不在内部”，
// 那种情况由边分支继续处理。
func originInTriangle(a, b, c Point) bool {
	area := b.sub(a).cross(c.sub(a))
	scale := math.Max(1.0, math.Max(a.length(), math.Max(b.length(), c.length())))
	tol := EpsAbs * scale * scale
	if math.Abs(area) <= tol {
		return false
	}
	c1 := a.cross(b) // 边 a->b 与原点
	c2 := b.cross(c)
	c3 := c.cross(a)
	if area > 0 {
		return c1 >= -tol && c2 >= -tol && c3 >= -tol
	}
	return c1 <= tol && c2 <= tol && c3 <= tol
}
