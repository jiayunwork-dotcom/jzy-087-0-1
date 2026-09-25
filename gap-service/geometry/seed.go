package geometry

import "math"

// buildSeedHull 构造 EPA 的初始凸包（必须是逆时针、且包住原点的凸多边形）。
//
// 常规穿透：GJK 收敛时已经得到一个包住原点的三角形，只需校正环绕方向。
// 零距离接触：单纯形退化成过原点的线段 / 点，这里沿坐标轴与边法向补充
// 若干支撑点，再从中挑出包住原点的逆时针三角形作为 EPA 种子。
// 理论上两个合法凸多边形必然能给出这样的三角形；极端情况下找不到时，
// 返回一个零深度的接触结论（法向取边的法向），仍然属于穿透分支，
// 不会把接触错报成“距离为零的分离”。
func buildSeedHull(a, b []Point, seed []MinkowskiPoint, touch bool) ([]MinkowskiPoint, error) {
	pts := make([]MinkowskiPoint, 0, 8)
	index := map[[2]int64]int{} // 以量化坐标去重
	quantum := func(p Point) [2]int64 {
		q := EpsAbs * math.Max(1, polygonScaleFromPoints(a, b))
		return [2]int64{int64(math.Round(p.X / q)), int64(math.Round(p.Y / q))}
	}
	add := func(p MinkowskiPoint) {
		key := quantum(p.V)
		if _, ok := index[key]; !ok {
			index[key] = len(pts)
			pts = append(pts, p)
		}
	}
	for _, s := range seed {
		add(s)
	}

	// 常规情形：三角形种子，校正为逆时针后直接返回。
	if len(pts) == 3 && !touch {
		if area2(pts[0].V, pts[1].V, pts[2].V) < 0 {
			pts[1], pts[2] = pts[2], pts[1]
		}
		return pts, nil
	}

	// 接触退化（或任何不足三点的情形）：补充支撑方向。
	dirs := []Point{{X: 1}, {X: -1}, {Y: 1}, {Y: -1}}
	for i := 0; i < len(seed); i++ {
		j := (i + 1) % len(seed)
		if len(seed) < 2 {
			break
		}
		n := leftNormal(seed[i].V, seed[j].V)
		if n.len2() > EpsAbs*EpsAbs {
			dirs = append(dirs, n.normalized(), n.normalized().neg())
		}
	}
	for _, d := range dirs {
		add(supportMinkowski(a, b, d))
	}

	// 从候选点中枚举第一个包住原点的逆时针三角形。
	if tri := pickContainingTriangle(pts); tri != nil {
		return tri, nil
	}
	return nil, errNoSeedTriangle
}

// errNoSeedTriangle 表示种子多面体无法包住原点（仅在数值极端的接触情形出现）。
var errNoSeedTriangle = &seedError{}

type seedError struct{}

func (*seedError) Error() string {
	return "EPA could not build a seed hull enclosing the origin (zero-depth contact fallback)"
}

func polygonScaleFromPoints(a, b []Point) float64 {
	return math.Max(polygonScale(a), polygonScale(b))
}

func area2(a, b, c Point) float64 { return b.sub(a).cross(c.sub(a)) }

// pickContainingTriangle 在候选点中找一组逆时针且（含边界地）包住原点的三点。
func pickContainingTriangle(pts []MinkowskiPoint) []MinkowskiPoint {
	n := len(pts)
	for i := 0; i < n; i++ {
		for j := i + 1; j < n; j++ {
			for k := j + 1; k < n; k++ {
				tri := []MinkowskiPoint{pts[i], pts[j], pts[k]}
				ar := area2(tri[0].V, tri[1].V, tri[2].V)
				scale := math.Max(1, math.Max(tri[0].V.length(),
					math.Max(tri[1].V.length(), tri[2].V.length())))
				if math.Abs(ar) <= EpsAbs*scale*scale {
					continue
				}
				if ar < 0 {
					tri[1], tri[2] = tri[2], tri[1]
				}
				if originInTriangle(tri[0].V, tri[1].V, tri[2].V) {
					return tri
				}
			}
		}
	}
	return nil
}
