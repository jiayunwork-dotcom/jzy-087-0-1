package geometry_test

import (
	"errors"
	"math"
	"testing"

	"github.com/assembly-gap/gap-service/geometry"
)

// 测试中统一使用的数值容差。几何判据内核内部用绝对+相对容差（1e-9 量级），
// 对外断言取 1e-8，既严格又不依赖浮点逐位一致。
const tol = 1e-8

func approx(x, y float64) bool { return math.Abs(x-y) <= tol }

func pt(x, y float64) geometry.Point { return geometry.Point{X: x, Y: y} }

func add(p, q geometry.Point) geometry.Point { return geometry.Point{X: p.X + q.X, Y: p.Y + q.Y} }
func mul(p geometry.Point, s float64) geometry.Point {
	return geometry.Point{X: p.X * s, Y: p.Y * s}
}
func vlen(p geometry.Point) float64 { return math.Sqrt(p.X*p.X + p.Y*p.Y) }

// rectCCW 返回以 (cx,cy) 为中心、半宽 hw、半高 hh 的逆时针轴对齐矩形。
func rectCCW(cx, cy, hw, hh float64) []geometry.Point {
	return []geometry.Point{
		pt(cx-hw, cy-hh),
		pt(cx+hw, cy-hh),
		pt(cx+hw, cy+hh),
		pt(cx-hw, cy+hh),
	}
}

func translate(poly []geometry.Point, d geometry.Point) []geometry.Point {
	out := make([]geometry.Point, len(poly))
	for i, p := range poly {
		out[i] = add(p, d)
	}
	return out
}

// ---------- 非法与退化输入 ----------

func TestValidate_TooFewVertices(t *testing.T) {
	cases := [][]geometry.Point{
		nil,
		{},
		{pt(0, 0)},
		{pt(0, 0), pt(1, 0)},
	}
	for i, poly := range cases {
		if err := geometry.ValidatePolygon(poly); !errors.Is(err, geometry.ErrTooFewVertices) {
			t.Fatalf("case %d: want ErrTooFewVertices, got %v", i, err)
		}
	}
}

func TestValidate_Degenerate(t *testing.T) {
	// 三点共线：面积退化为零。
	collinear := []geometry.Point{pt(0, 0), pt(1, 0), pt(2, 0)}
	if err := geometry.ValidatePolygon(collinear); !errors.Is(err, geometry.ErrZeroArea) {
		t.Fatalf("collinear: want ErrZeroArea, got %v", err)
	}

	// 相邻顶点重合。
	dup := []geometry.Point{pt(0, 0), pt(0, 0), pt(1, 0), pt(1, 1)}
	if err := geometry.ValidatePolygon(dup); err == nil {
		t.Fatalf("duplicate consecutive vertex should be rejected")
	}
}

// ---------- 非凸直接拒绝（不做凸分解） ----------

func TestValidate_NonConvexRejected(t *testing.T) {
	// 箭头形凹多边形（单位正方形右上角向内凹）。
	concave := []geometry.Point{
		pt(0, 0), pt(2, 0), pt(2, 1), pt(1, 1), pt(1, 2), pt(0, 2),
	}
	if err := geometry.ValidatePolygon(concave); !errors.Is(err, geometry.ErrNonConvex) {
		t.Fatalf("concave polygon: want ErrNonConvex, got %v", err)
	}

	// 对凹多边形调用查询同样必须拒绝，不能偷偷凸包化。
	other := rectCCW(5, 5, 1, 1)
	if _, err := geometry.Query(concave, other); err == nil {
		t.Fatalf("Query on a concave polygon must be rejected")
	}
}

func TestValidate_ConvexWithCollinearPointsAccepted(t *testing.T) {
	// 凸矩形边上插入一个共线冗余顶点，仍是合法凸多边形。
	withCollinear := []geometry.Point{
		pt(0, 0), pt(1, 0), pt(2, 0), pt(2, 2), pt(0, 2),
	}
	if err := geometry.ValidatePolygon(withCollinear); err != nil {
		t.Fatalf("convex polygon with collinear vertex should be accepted: %v", err)
	}
}

// ---------- 间隙已知算例：轴对齐、缝隙确定的两个矩形 ----------
//
//	A: [-4,-1] x [-1,1]  （右边界 x=-1）
//	B: [ 2, 4] x [-1,1]  （左边界 x= 2）
//
// 边到边间隔 = 3，接触法向 = +x。
func TestQuery_AxisAlignedRectangles_KnownGap(t *testing.T) {
	a := rectCCW(-2.5, 0, 1.5, 1)
	b := rectCCW(3, 0, 1, 1)

	res, err := geometry.Query(a, b)
	if err != nil {
		t.Fatalf("Query failed: %v", err)
	}
	if res.Status != geometry.StatusSeparated {
		t.Fatalf("status = %s, want separated", res.Status)
	}
	if !approx(res.Distance, 3.0) {
		t.Fatalf("distance = %.12f, want exactly 3 (edge-to-edge gap)", res.Distance)
	}
	if !approx(res.Normal.X, 1) || !approx(res.Normal.Y, 0) {
		t.Fatalf("normal = (%.6f,%.6f), want (1,0)", res.Normal.X, res.Normal.Y)
	}
	// 最近点分别落在两条相对的竖直边上。
	if !approx(res.PointOnA.X, -1) {
		t.Fatalf("point on A x = %.6f, want -1", res.PointOnA.X)
	}
	if !approx(res.PointOnB.X, 2) {
		t.Fatalf("point on B x = %.6f, want 2", res.PointOnB.X)
	}
	if math.Abs(res.PointOnA.Y-res.PointOnB.Y) > tol {
		t.Fatalf("closest points should share y for parallel edges, got %v vs %v",
			res.PointOnA, res.PointOnB)
	}
	if math.Abs(vlen(res.Normal)-1) > tol {
		t.Fatalf("normal not unit length: %v", res.Normal)
	}
}

// ---------- 沿分离方向平移零件，最近距离等量增减 ----------

func TestQuery_TranslateAlongNormal_DistanceChangesBySameAmount(t *testing.T) {
	a := rectCCW(-2.5, 0, 1.5, 1)
	b0 := rectCCW(3, 0, 1, 1)

	base, err := geometry.Query(a, b0)
	if err != nil || base.Status != geometry.StatusSeparated {
		t.Fatalf("base query: %+v err=%v", base, err)
	}

	// 把 B 沿接触法向往远离 A 的方向移动 s，间隙应增加 s。
	for _, s := range []float64{0.25, 1.0, 3.7} {
		moved := translate(b0, mul(base.Normal, s))
		res, err := geometry.Query(a, moved)
		if err != nil {
			t.Fatalf("s=%v: %v", s, err)
		}
		if want := base.Distance + s; !approx(res.Distance, want) {
			t.Fatalf("moving B away by %.2f: distance %.12f, want %.12f", s, res.Distance, want)
		}
	}

	// 沿法向往 A 移动 s（仍分离），间隙应等量减小。
	for _, s := range []float64{0.5, 1.5, 2.9} {
		moved := translate(b0, mul(base.Normal, -s))
		res, err := geometry.Query(a, moved)
		if err != nil {
			t.Fatalf("s=%v: %v", s, err)
		}
		if res.Status != geometry.StatusSeparated {
			t.Fatalf("s=%v: still separated expected, got %s", s, res.Status)
		}
		if want := base.Distance - s; !approx(res.Distance, want) {
			t.Fatalf("moving B closer by %.2f: distance %.12f, want %.12f", s, res.Distance, want)
		}
	}
}

// 沿垂直于法向的方向平移，间隙不变（面接触宽度足够，仍是边对边）。
func TestQuery_TranslatePerpendicularToNormal_GapUnchanged(t *testing.T) {
	a := rectCCW(-2.5, 0, 1.5, 2)
	b := rectCCW(3, 0, 1, 2)
	base, err := geometry.Query(a, b)
	if err != nil {
		t.Fatal(err)
	}
	for _, dy := range []float64{0.3, -0.7, 1.0} {
		res, err := geometry.Query(a, translate(b, pt(0, dy)))
		if err != nil {
			t.Fatalf("dy=%v: %v", dy, err)
		}
		if !approx(res.Distance, base.Distance) {
			t.Fatalf("dy=%v: distance changed %.12f -> %.12f", dy, base.Distance, res.Distance)
		}
	}
}

// ---------- 两体同时做相同平移，间隙保持不变 ----------

func TestQuery_RigidTranslationBoth_GapInvariant(t *testing.T) {
	a0 := rectCCW(-2.5, 0, 1.5, 1)
	b0 := rectCCW(3, 0, 1, 1)

	shifts := []geometry.Point{
		pt(5, 0), pt(-3.2, 7.6), pt(100, -50),
	}
	for _, s := range shifts {
		base, err := geometry.Query(a0, b0)
		if err != nil {
			t.Fatal(err)
		}
		res, err := geometry.Query(translate(a0, s), translate(b0, s))
		if err != nil {
			t.Fatalf("shift %v: %v", s, err)
		}
		if res.Status != base.Status || !approx(res.Distance, base.Distance) {
			t.Fatalf("shift %v: gap changed %+v -> %+v", s, base, res)
		}
		if !approx(res.Normal.X, base.Normal.X) || !approx(res.Normal.Y, base.Normal.Y) {
			t.Fatalf("shift %v: normal changed %v -> %v", s, base.Normal, res.Normal)
		}
	}
}

// 穿透分支也要满足“整体平移不变”。
func TestQuery_Penetration_RigidTranslationInvariant(t *testing.T) {
	// A：x∈[-1,1]、y∈[-1.5,1.5]；B 中心 1.7 半宽 1（x∈[0.7,2.7]）：
	// x 向重叠 0.3（较小），y 向重叠 2.0。
	a := rectCCW(0, 0, 1, 1.5)
	b := rectCCW(1.7, 0, 1, 1)

	base, err := geometry.Query(a, b)
	if err != nil {
		t.Fatal(err)
	}
	if base.Status != geometry.StatusPenetrating {
		t.Fatalf("status = %s, want penetrating", base.Status)
	}
	for _, s := range []geometry.Point{pt(10, 10), pt(-4, 2.2)} {
		res, err := geometry.Query(translate(a, s), translate(b, s))
		if err != nil {
			t.Fatal(err)
		}
		if !approx(res.Depth, base.Depth) {
			t.Fatalf("shift %v: depth changed %.12f -> %.12f", s, base.Depth, res.Depth)
		}
		if !approx(res.Normal.X, base.Normal.X) || !approx(res.Normal.Y, base.Normal.Y) {
			t.Fatalf("shift %v: normal changed %v -> %v", s, base.Normal, res.Normal)
		}
	}
}

// ---------- 重叠量已知的矩形：穿透深度 = 较小的重叠尺寸 ----------

func TestQuery_OverlappingRectangles_DepthIsSmallerOverlap(t *testing.T) {
	// x 向重叠 0.3（较小），y 向重叠 2.0（较大），深度必须是 0.3。
	a := rectCCW(0, 0, 1, 1.5)
	b := rectCCW(1.7, 0, 1, 1)

	res, err := geometry.Query(a, b)
	if err != nil {
		t.Fatal(err)
	}
	if res.Status != geometry.StatusPenetrating {
		t.Fatalf("status = %s, want penetrating", res.Status)
	}
	if !approx(res.Depth, 0.3) {
		t.Fatalf("penetration depth = %.12f, want 0.3 (smaller overlap)", res.Depth)
	}
	// 推开方向沿较小重叠的轴：x 向，B 应被推向 +x。
	if !approx(math.Abs(res.Normal.X), 1) || !approx(math.Abs(res.Normal.Y), 0) {
		t.Fatalf("normal = %v, want (±1,0)", res.Normal)
	}
	if res.Normal.X < 0 {
		t.Fatalf("normal must point from A toward B (push B away), got %v", res.Normal)
	}
	if math.Abs(vlen(res.Normal)-1) > tol {
		t.Fatalf("normal not unit: %v", res.Normal)
	}

	// 沿该法向恰好移动 depth 后，两体应正好分开（距离 ≈ 0 的接触，
	// 再多移动一个小量即给出正间隙且随位移线性增长）。
	separated := translate(b, mul(res.Normal, res.Depth))
	just, err := geometry.Query(a, separated)
	if err != nil {
		t.Fatal(err)
	}
	if just.Status == geometry.StatusSeparated && just.Distance > 1e-6 {
		t.Fatalf("after moving exactly depth, bodies should be in contact, got gap %.9f", just.Distance)
	}

	extra := 0.5
	gapped := translate(b, mul(res.Normal, res.Depth+extra))
	r2, err := geometry.Query(a, gapped)
	if err != nil {
		t.Fatal(err)
	}
	if r2.Status != geometry.StatusSeparated || !approx(r2.Distance, extra) {
		t.Fatalf("after extra %.1f of separation: %+v", extra, r2)
	}
}

// 两个重叠尺寸相等（正方重叠）：深度取该公共重叠量。
func TestQuery_OverlappingSquares_EqualOverlap(t *testing.T) {
	a := rectCCW(0, 0, 1, 1)
	b := rectCCW(1.5, 0, 1, 1) // x 向重叠 0.5
	res, err := geometry.Query(a, b)
	if err != nil {
		t.Fatal(err)
	}
	if !approx(res.Depth, 0.5) {
		t.Fatalf("depth = %.12f, want 0.5", res.Depth)
	}
}

// ---------- 恰好接触不是“距离为零的分离” ----------

func TestQuery_TouchingRectangles_ReportedAsContact(t *testing.T) {
	a := rectCCW(0, 0, 1, 1) // 右边界 x=1
	b := rectCCW(2, 0, 1, 1) // 左边界 x=1，恰好贴合
	res, err := geometry.Query(a, b)
	if err != nil {
		t.Fatal(err)
	}
	if res.Status != geometry.StatusPenetrating {
		t.Fatalf("touching must be reported as penetration/contact, got %s", res.Status)
	}
	if !approx(res.Depth, 0) {
		t.Fatalf("contact depth = %v, want 0", res.Depth)
	}
	if math.Abs(vlen(res.Normal)-1) > tol {
		t.Fatalf("contact normal not unit: %v", res.Normal)
	}
}

// 一个多边形完全包含另一个：EPA 仍应给出把内多边形推出所需的最小深度。
func TestQuery_ContainedPolygon(t *testing.T) {
	outer := rectCCW(0, 0, 5, 5)
	inner := rectCCW(0, 0, 1, 2)
	res, err := geometry.Query(outer, inner)
	if err != nil {
		t.Fatal(err)
	}
	if res.Status != geometry.StatusPenetrating {
		t.Fatalf("contained polygon must be penetration, got %s", res.Status)
	}
	// 内矩形要完全移出外矩形：内左边 (-1) 跨出外右边 (5) 需要位移 6；
	// 内下边 (-2) 跨出外上边 (5) 需要 7；最小推出量为 6（沿 ±x）。
	if !approx(res.Depth, 6) {
		t.Fatalf("depth = %.12f, want 6", res.Depth)
	}
	if !approx(math.Abs(res.Normal.Y), 0) {
		t.Fatalf("normal = %v, want along x axis", res.Normal)
	}
}

// ---------- 迭代上限：用尽必须报错，不能静默返回 ----------

func TestQuery_GJKIterationLimitReturnsError(t *testing.T) {
	old := geometry.MaxGJKSteps
	geometry.MaxGJKSteps = 0
	defer func() { geometry.MaxGJKSteps = old }()

	a := rectCCW(-2.5, 0, 1.5, 1)
	b := rectCCW(3, 0, 1, 1)
	if _, err := geometry.Query(a, b); !errors.Is(err, geometry.ErrGJKNotConverged) {
		t.Fatalf("want ErrGJKNotConverged, got %v", err)
	}
}

func TestQuery_EPAIterationLimitReturnsError(t *testing.T) {
	old := geometry.MaxEPASteps
	geometry.MaxEPASteps = 0
	defer func() { geometry.MaxEPASteps = old }()

	a := rectCCW(0, 0, 1, 1.5)
	b := rectCCW(1.7, 0, 1, 1)
	if _, err := geometry.Query(a, b); !errors.Is(err, geometry.ErrEPANotConverged) {
		t.Fatalf("want ErrEPANotConverged, got %v", err)
	}
}
