package geometry

import (
	"errors"
	"math"
)

// 校验失败原因。本服务只处理凸体：顶点不足、退化、非凸都直接拒绝，
// 不做任何凸包近似或凸分解。
var (
	ErrTooFewVertices = errors.New("polygon must contain at least 3 vertices")
	ErrNonFinite      = errors.New("polygon contains a non-finite (NaN/Inf) coordinate")
	ErrDuplicatePoint = errors.New("polygon contains duplicate consecutive vertices")
	ErrZeroArea       = errors.New("polygon area is zero: the polygon is degenerate")
	ErrNonConvex      = errors.New("polygon is not convex; only convex polygons are supported and no convex decomposition is performed")
)

// polygonScale 取顶点坐标量级，用于构造“绝对 + 相对”容差。
func polygonScale(poly []Point) float64 {
	m := 1.0
	for _, p := range poly {
		if a := math.Abs(p.X); a > m {
			m = a
		}
		if a := math.Abs(p.Y); a > m {
			m = a
		}
	}
	return m
}

// ValidatePolygon 校验入参多边形：
//  1. 顶点数不少于 3；
//  2. 坐标均为有限值；
//  3. 相邻顶点不重合；
//  4. 面积不为零（用鞋带公式）；
//  5. 顶点按一致顺序（顺时针或逆时针）环绕且每个内角不超过 π —— 即凸。
//     允许边上有共线的冗余顶点；一旦出现方向反转（叉积变号）即判凹。
func ValidatePolygon(poly []Point) error {
	if len(poly) < 3 {
		return ErrTooFewVertices
	}

	for _, p := range poly {
		if math.IsNaN(p.X) || math.IsNaN(p.Y) || math.IsInf(p.X, 0) || math.IsInf(p.Y, 0) {
			return ErrNonFinite
		}
	}

	n := len(poly)
	tol := EpsAbs * polygonScale(poly)

	for i := 0; i < n; i++ {
		if poly[i].sub(poly[(i+1)%n]).len2() <= tol*tol {
			return ErrDuplicatePoint
		}
	}

	// 鞋带公式：面积退化（所有顶点共线）直接拒绝。
	var area2 float64
	for i := 0; i < n; i++ {
		q := poly[(i+1)%n]
		area2 += poly[i].cross(q)
	}
	if math.Abs(area2) <= tol*tol {
		return ErrZeroArea
	}

	// 凸性：相邻边叉积符号必须一致。允许叉积为 0（共线顶点）。
	var sign float64
	for i := 0; i < n; i++ {
		e1 := poly[i].sub(poly[(i+n-1)%n])
		e2 := poly[(i+1)%n].sub(poly[i])
		c := e1.cross(e2)
		if math.Abs(c) <= tol*tol {
			continue
		}
		s := math.Copysign(1, c)
		if sign == 0 {
			sign = s
		} else if s != sign {
			return ErrNonConvex
		}
	}
	return nil
}
