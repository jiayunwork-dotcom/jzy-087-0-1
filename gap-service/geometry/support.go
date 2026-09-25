package geometry

import "math"

// ---------- 基本向量运算 ----------

func (p Point) add(q Point) Point { return Point{p.X + q.X, p.Y + q.Y} }
func (p Point) sub(q Point) Point { return Point{p.X - q.X, p.Y - q.Y} }
func (p Point) scale(s float64) Point {
	return Point{p.X * s, p.Y * s}
}
func (p Point) dot(q Point) float64 { return p.X*q.X + p.Y*q.Y }

// cross 返回二维叉积 p.x*q.y - p.y*q.x：
//   - >0 时 q 在 p 的逆时针（左侧）方向；
//   - <0 时 q 在 p 的顺时针（右侧）方向。
func (p Point) cross(q Point) float64 { return p.X*q.Y - p.Y*q.X }

func (p Point) len2() float64   { return p.X*p.X + p.Y*p.Y }
func (p Point) length() float64 { return math.Sqrt(p.len2()) }

func (p Point) neg() Point { return Point{-p.X, -p.Y} }

func (p Point) normalized() Point {
	l := p.length()
	if l <= EpsAbs {
		return Point{}
	}
	return Point{p.X / l, p.Y / l}
}

// leftNormal 返回线段 p->q 的左侧法向（多边形逆时针环绕时的外法向）。
func leftNormal(p, q Point) Point {
	d := q.sub(p)
	return Point{d.Y, -d.X}
}

// ---------- 支撑函数 ----------
//
// supportFarthest 沿方向 d 返回多边形上投影最大的顶点（GJK 的支撑映射）。
// 投影相同时取下标最小者，保证结果确定、可复核。
func supportFarthest(poly []Point, d Point) (Point, int) {
	bestIdx := 0
	bestVal := poly[0].dot(d)
	for i := 1; i < len(poly); i++ {
		if v := poly[i].dot(d); v > bestVal {
			bestVal = v
			bestIdx = i
		}
	}
	return poly[bestIdx], bestIdx
}

// supportMinkowski 返回 Minkowski 差 A⊖B 沿方向 d 的支撑点：
//
//	S_A(d) - S_B(-d)
//
// 同时记录两个源多边形上的支撑顶点 S_A(d) 与 S_B(-d)，
// 之后可利用单纯形的重心坐标把最近点还原成两体上的最近点对。
func supportMinkowski(a, b []Point, d Point) MinkowskiPoint {
	sa, _ := supportFarthest(a, d)
	sb, _ := supportFarthest(b, d.neg())
	return MinkowskiPoint{V: sa.sub(sb), A: sa, B: sb}
}
