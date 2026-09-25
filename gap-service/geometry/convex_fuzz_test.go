package geometry_test

import (
	"math"
	"math/rand"
	"sort"
	"testing"

	"github.com/assembly-gap/gap-service/geometry"
)

// convexHull 用 Andrew 单调链算法从随机点生成严格凸多边形（CCW）。
func convexHull(points []geometry.Point) []geometry.Point {
	ps := append([]geometry.Point(nil), points...)
	sort.Slice(ps, func(i, j int) bool {
		if ps[i].X != ps[j].X {
			return ps[i].X < ps[j].X
		}
		return ps[i].Y < ps[j].Y
	})
	cross := func(o, a, b geometry.Point) float64 {
		return (a.X-o.X)*(b.Y-o.Y) - (a.Y-o.Y)*(b.X-o.X)
	}
	var lower, upper []geometry.Point
	for _, p := range ps {
		for len(lower) >= 2 && cross(lower[len(lower)-2], lower[len(lower)-1], p) <= 0 {
			lower = lower[:len(lower)-1]
		}
		lower = append(lower, p)
	}
	for i := len(ps) - 1; i >= 0; i-- {
		p := ps[i]
		for len(upper) >= 2 && cross(upper[len(upper)-2], upper[len(upper)-1], p) <= 0 {
			upper = upper[:len(upper)-1]
		}
		upper = append(upper, p)
	}
	return append(lower[:len(lower)-1], upper[:len(upper)-1]...)
}

func randomConvexPolygon(rng *rand.Rand) []geometry.Point {
	n := 5 + rng.Intn(8)
	pts := make([]geometry.Point, n)
	for i := range pts {
		pts[i] = pt(rng.Float64()*2-1, rng.Float64()*2-1)
	}
	return convexHull(pts)
}

// pointSegmentDist 点到线段的距离。
func pointSegmentDist(p, a, b geometry.Point) float64 {
	d := sub(b, a)
	l2 := d.X*d.X + d.Y*d.Y
	t := (p.X-a.X)*d.X + (p.Y-a.Y)*d.Y
	if l2 > 0 {
		t /= l2
	}
	t = math.Max(0, math.Min(1, t))
	proj := geometry.Point{X: a.X + t*d.X, Y: a.Y + t*d.Y}
	return vlen(sub(p, proj))
}

// bruteDistance 枚举所有「顶点-对边」距离，精确求两个凸多边形的最近距离。
func bruteDistance(a, b []geometry.Point) float64 {
	best := math.Inf(1)
	for _, p := range a {
		for j := range b {
			best = math.Min(best, pointSegmentDist(p, b[j], b[(j+1)%len(b)]))
		}
	}
	for _, p := range b {
		for j := range a {
			best = math.Min(best, pointSegmentDist(p, a[j], a[(j+1)%len(a)]))
		}
	}
	return best
}

// satResult 用分离轴定理给出参考结论：
// 分离时 value 为沿最近法向的最小推出间距（仅作重叠判定与方向参考，
// 欧氏距离仍由 bruteDistance 精确给出）；穿透时 value 为最小穿透深度。
type satResult struct {
	overlap bool
	value   float64
	nx, ny  float64
}

func satReference(a, b []geometry.Point) satResult {
	type axis struct{ x, y float64 }
	var axes []axis
	addAxes := func(poly []geometry.Point) {
		for i := range poly {
			p, q := poly[i], poly[(i+1)%len(poly)]
			e := sub(q, p)
			l := math.Hypot(e.X, e.Y)
			axes = append(axes, axis{-e.Y / l, e.X / l})
		}
	}
	addAxes(a)
	addAxes(b)

	project := func(poly []geometry.Point, ax axis) (float64, float64) {
		lo, hi := math.Inf(1), math.Inf(-1)
		for _, p := range poly {
			s := p.X*ax.x + p.Y*ax.y
			lo = math.Min(lo, s)
			hi = math.Max(hi, s)
		}
		return lo, hi
	}
	centroid := func(poly []geometry.Point) (cx, cy float64) {
		for _, p := range poly {
			cx += p.X
			cy += p.Y
		}
		return cx / float64(len(poly)), cy / float64(len(poly))
	}
	acx, acy := centroid(a)
	bcx, bcy := centroid(b)

	minPenetration := math.Inf(1)
	var penN axis
	gap := 0.0
	separated := false
	for _, ax := range axes {
		amin, amax := project(a, ax)
		bmin, bmax := project(b, ax)
		if bmin > amax {
			if !separated || bmin-amax < gap {
				separated = true
				gap = bmin - amax
			}
			continue
		}
		if amin > bmax {
			if !separated || amin-bmax < gap {
				separated = true
				gap = amin - bmax
			}
			continue
		}
		overlap := math.Min(amax-bmin, bmax-amin)
		if overlap < minPenetration {
			minPenetration = overlap
			// 方向：把 B 沿轴推离 A（轴取由 A 指向 B 的一侧）。
			dot := (bcx-acx)*ax.x + (bcy-acy)*ax.y
			s := 1.0
			if dot < 0 {
				s = -1
			}
			penN = axis{ax.x * s, ax.y * s}
		}
	}
	if separated {
		return satResult{overlap: false, value: gap}
	}
	return satResult{overlap: true, value: minPenetration, nx: penN.x, ny: penN.y}
}

func TestQuery_RandomConvexPolygons_ReferenceCrossCheck(t *testing.T) {
	rng := rand.New(rand.NewSource(99887766))
	const n = 600
	var nSep, nPen int
	for iter := 0; iter < n; iter++ {
		a := randomConvexPolygon(rng)
		b := randomConvexPolygon(rng)

		// 随机整体变换：平移 + 均匀缩放后再加可控相对位移。
		scaleA := 0.3 + rng.Float64()*2.5
		scaleB := 0.3 + rng.Float64()*2.5
		for i := range a {
			a[i] = geometry.Point{X: a[i].X * scaleA, Y: a[i].Y * scaleA}
		}
		for i := range b {
			b[i] = geometry.Point{X: b[i].X * scaleB, Y: b[i].Y * scaleB}
		}
		b = rotatePoly(b, rng.Float64()*2*math.Pi)
		shift := pt((rng.Float64()-0.5)*7, (rng.Float64()-0.5)*7)
		b = translate(b, shift)
		// 平移到非零原点，顺带核对平移不变性。
		originShift := pt(rng.Float64()*40-20, rng.Float64()*40-20)
		a = translate(a, originShift)
		b = translate(b, originShift)

		ref := satReference(a, b)
		got, err := geometry.Query(a, b)
		if err != nil {
			t.Fatalf("iter %d: %v", iter, err)
		}

		if !ref.overlap {
			nSep++
			if got.Status != geometry.StatusSeparated {
				t.Fatalf("iter %d: got %s want separated", iter, got.Status)
			}
			want := bruteDistance(a, b)
			if math.Abs(got.Distance-want) > 1e-6*math.Max(1, want) {
				t.Fatalf("iter %d: distance %.9f brute %.9f", iter, got.Distance, want)
			}
			if math.Abs(vlen(got.Normal)-1) > 1e-9 {
				t.Fatalf("iter %d: normal not unit", iter)
			}
		} else {
			nPen++
			if got.Status != geometry.StatusPenetrating {
				t.Fatalf("iter %d: got %s want penetrating", iter, got.Status)
			}
			if math.Abs(got.Depth-ref.value) > 1e-6*math.Max(1, ref.value) {
				t.Fatalf("iter %d: depth %.9f SAT %.9f", iter, got.Depth, ref.value)
			}
			if math.Abs(got.Normal.X-ref.nx) > 1e-6 || math.Abs(got.Normal.Y-ref.ny) > 1e-6 {
				t.Fatalf("iter %d: normal (%.6f,%.6f) SAT (%.6f,%.6f)",
					iter, got.Normal.X, got.Normal.Y, ref.nx, ref.ny)
			}
		}
	}
	if nSep == 0 || nPen == 0 {
		t.Fatalf("bad fuzz coverage: separated=%d penetrating=%d", nSep, nPen)
	}
	t.Logf("cross-check coverage: %d separated, %d penetrating", nSep, nPen)
}
