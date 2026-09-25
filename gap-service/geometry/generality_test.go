package geometry_test

import (
	"math"
	"testing"

	"github.com/assembly-gap/gap-service/geometry"
)

func rotate(p geometry.Point, theta float64) geometry.Point {
	c, s := math.Cos(theta), math.Sin(theta)
	return geometry.Point{X: c*p.X - s*p.Y, Y: s*p.X + c*p.Y}
}

func rotatePoly(poly []geometry.Point, theta float64) []geometry.Point {
	out := make([]geometry.Point, len(poly))
	for i, p := range poly {
		out[i] = rotate(p, theta)
	}
	return out
}

// 对角放置的两个正方形：最近特征是角点-角点，距离为 2√2，法向沿 (1,1)。
func TestQuery_VertexVertex_CornerGap(t *testing.T) {
	a := rectCCW(0, 0, 1, 1)
	b := rectCCW(4, 4, 1, 1)
	res, err := geometry.Query(a, b)
	if err != nil {
		t.Fatal(err)
	}
	if res.Status != geometry.StatusSeparated {
		t.Fatalf("status=%s", res.Status)
	}
	want := 2 * math.Sqrt2
	if math.Abs(res.Distance-want) > 1e-8 {
		t.Fatalf("distance=%.10f want %.10f", res.Distance, want)
	}
	wantN := 1 / math.Sqrt2
	if math.Abs(res.Normal.X-wantN) > 1e-8 || math.Abs(res.Normal.Y-wantN) > 1e-8 {
		t.Fatalf("normal=%v want (%.6f,%.6f)", res.Normal, wantN, wantN)
	}
	if math.Abs(res.PointOnA.X-1) > 1e-8 || math.Abs(res.PointOnA.Y-1) > 1e-8 {
		t.Fatalf("point on A=%v want (1,1)", res.PointOnA)
	}
	if math.Abs(res.PointOnB.X-3) > 1e-8 || math.Abs(res.PointOnB.Y-3) > 1e-8 {
		t.Fatalf("point on B=%v want (3,3)", res.PointOnB)
	}
}

// 同时旋转两个多边形：距离/深度不变，法向跟着旋转。
func TestQuery_RotationInvariance(t *testing.T) {
	a0 := rectCCW(0, 0, 1, 1.5)
	b0 := rectCCW(1.7, 0, 1, 1)
	for _, theta := range []float64{0.17, 0.9, math.Pi / 3, 1.7} {
		base, err := geometry.Query(a0, b0)
		if err != nil {
			t.Fatal(err)
		}
		res, err := geometry.Query(rotatePoly(a0, theta), rotatePoly(b0, theta))
		if err != nil {
			t.Fatalf("theta=%.2f: %v", theta, err)
		}
		if math.Abs(res.Depth-base.Depth) > 1e-7 {
			t.Fatalf("theta=%.2f: depth %.10f != %.10f", theta, res.Depth, base.Depth)
		}
		wantN := rotate(base.Normal, theta)
		if math.Abs(res.Normal.X-wantN.X) > 1e-7 || math.Abs(res.Normal.Y-wantN.Y) > 1e-7 {
			t.Fatalf("theta=%.2f: normal %v want %v", theta, res.Normal, wantN)
		}
	}

	// 分离情形同样旋转不变。
	c0 := rectCCW(0, 0, 1, 1)
	d0 := rectCCW(4, 4, 1, 1)
	for _, theta := range []float64{0.4, 1.3} {
		base, err := geometry.Query(c0, d0)
		if err != nil {
			t.Fatal(err)
		}
		res, err := geometry.Query(rotatePoly(c0, theta), rotatePoly(d0, theta))
		if err != nil {
			t.Fatal(err)
		}
		if math.Abs(res.Distance-base.Distance) > 1e-7 {
			t.Fatalf("theta=%.2f: distance %.10f != %.10f", theta, res.Distance, base.Distance)
		}
	}
}

// 非轴对齐的三角形：中心在原点的单位正三角与向右偏移的三角形，
// 分离距离与法向用支撑点关系交叉核对。
func TestQuery_Triangles(t *testing.T) {
	tri := func(cx, cy, r float64) []geometry.Point {
		p := make([]geometry.Point, 3)
		for i := 0; i < 3; i++ {
			ang := -math.Pi/2 + float64(i)*2*math.Pi/3
			p[i] = pt(cx+r*math.Cos(ang), cy+r*math.Sin(ang))
		}
		return p
	}
	if err := geometry.ValidatePolygon(tri(0, 0, 1)); err != nil {
		t.Fatalf("triangle invalid: %v", err)
	}

	a := tri(0, 0, 1)
	b := tri(5, 0, 1)
	res, err := geometry.Query(a, b)
	if err != nil {
		t.Fatal(err)
	}
	// 正三角最右点为 (√3/2, 1/2)，两个相对尖点在 y=0.5 上水平相对：
	// 间隙 = 5 - 2·(√3/2) = 5-√3，法向 (1,0)。
	want := 5 - math.Sqrt(3)
	if math.Abs(res.Distance-want) > 1e-7 {
		t.Fatalf("distance=%.10f want %.10f", res.Distance, want)
	}
	if math.Abs(res.Normal.X-1) > 1e-7 || math.Abs(res.Normal.Y) > 1e-7 {
		t.Fatalf("normal=%v want (1,0)", res.Normal)
	}

	// 重叠三角形：MTD 后沿法向推开 depth+δ 必分离。
	b2 := tri(0.5, 0, 1)
	r2, err := geometry.Query(a, b2)
	if err != nil {
		t.Fatal(err)
	}
	if r2.Status != geometry.StatusPenetrating {
		t.Fatalf("status=%s want penetrating", r2.Status)
	}
	if r2.Depth <= 0 || r2.Depth > 2 {
		t.Fatalf("implausible depth %v", r2.Depth)
	}
	moved := translate(b2, mul(r2.Normal, r2.Depth+1e-6))
	r3, err := geometry.Query(a, moved)
	if err != nil {
		t.Fatal(err)
	}
	if r3.Status != geometry.StatusSeparated || r3.Distance > 1e-4 {
		t.Fatalf("after push-out: %+v", r3)
	}
}

// 顺时针环绕顺序也要正常工作。
func TestQuery_ClockwiseWinding(t *testing.T) {
	a := []geometry.Point{pt(0, 0), pt(0, 1), pt(1, 1), pt(1, 0)} // CW
	if err := geometry.ValidatePolygon(a); err != nil {
		t.Fatalf("CW rect rejected: %v", err)
	}
	b := rectCCW(4, 0, 1, 1)
	res, err := geometry.Query(a, b)
	if err != nil {
		t.Fatal(err)
	}
	if math.Abs(res.Distance-2) > 1e-8 {
		t.Fatalf("distance=%.10f want 2", res.Distance)
	}
}
