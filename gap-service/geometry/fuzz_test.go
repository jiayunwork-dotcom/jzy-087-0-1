package geometry_test

import (
	"math"
	"math/rand"
	"testing"

	"github.com/assembly-gap/gap-service/geometry"
)

func TestQuery_RandomRectangles_MatchAnalyticReference(t *testing.T) {
	rng := rand.New(rand.NewSource(20260925))
	const n = 400
	for iter := 0; iter < n; iter++ {
		ahw, ahh := 0.5+rng.Float64()*2.5, 0.5+rng.Float64()*2.5
		bhw, bhh := 0.5+rng.Float64()*2.5, 0.5+rng.Float64()*2.5
		acx, acy := (rng.Float64()-0.5)*4, (rng.Float64()-0.5)*4
		// 中心距离覆盖分离与穿透两种情形。
		bcx := acx + (rng.Float64()-0.5)*3.0*(ahw+bhw)
		bcy := acy + (rng.Float64()-0.5)*3.0*(ahh+bhh)

		a := rectCCW(acx, acy, ahw, ahh)
		b := rectCCW(bcx, bcy, bhw, bhh)

		want := analyticRect(acx, acy, ahw, ahh, bcx, bcy, bhw, bhh)
		got, err := geometry.Query(a, b)
		if err != nil {
			t.Fatalf("iter %d: query error: %v", iter, err)
		}

		if want.separated {
			if got.Status != geometry.StatusSeparated {
				t.Fatalf("iter %d: status=%s want separated", iter, got.Status)
			}
			if math.Abs(got.Distance-want.value) > 1e-7 {
				t.Fatalf("iter %d: distance=%.9f want %.9f", iter, got.Distance, want.value)
			}
			if math.Abs(got.Normal.X-want.nx) > 1e-7 || math.Abs(got.Normal.Y-want.ny) > 1e-7 {
				t.Fatalf("iter %d: normal=(%.6f,%.6f) want (%.6f,%.6f)",
					iter, got.Normal.X, got.Normal.Y, want.nx, want.ny)
			}
			// 最近点必须落在矩形内，且点对间距等于上报距离，连线沿法向。
			if !inRect(got.PointOnA, acx, acy, ahw, ahh) {
				t.Fatalf("iter %d: point on A outside: %v", iter, got.PointOnA)
			}
			if !inRect(got.PointOnB, bcx, bcy, bhw, bhh) {
				t.Fatalf("iter %d: point on B outside: %v", iter, got.PointOnB)
			}
			d := vlen(sub(got.PointOnB, got.PointOnA))
			if math.Abs(d-got.Distance) > 1e-7 {
				t.Fatalf("iter %d: pair distance %.9f != reported %.9f", iter, d, got.Distance)
			}
			pn := normalize(sub(got.PointOnB, got.PointOnA))
			if math.Abs(pn.X-got.Normal.X) > 1e-7 || math.Abs(pn.Y-got.Normal.Y) > 1e-7 {
				t.Fatalf("iter %d: closest-point direction %v != normal %v", iter, pn, got.Normal)
			}
		} else {
			if got.Status != geometry.StatusPenetrating {
				t.Fatalf("iter %d: status=%s want penetrating", iter, got.Status)
			}
			if math.Abs(got.Depth-want.value) > 1e-7 {
				t.Fatalf("iter %d: depth=%.9f want %.9f", iter, got.Depth, want.value)
			}
			if math.Abs(got.Normal.X-want.nx) > 1e-7 || math.Abs(got.Normal.Y-want.ny) > 1e-7 {
				t.Fatalf("iter %d: normal=(%.6f,%.6f) want (%.6f,%.6f)",
					iter, got.Normal.X, got.Normal.Y, want.nx, want.ny)
			}
			// 沿法向移动 depth+δ 后必然分离，且间隙 ≈ δ。
			for _, delta := range []float64{1e-6, 0.01, 0.5} {
				moved := translate(b, mul(got.Normal, got.Depth+delta))
				r, err := geometry.Query(a, moved)
				if err != nil {
					t.Fatalf("iter %d: post-separation query: %v", iter, err)
				}
				if r.Status != geometry.StatusSeparated || math.Abs(r.Distance-delta) > 1e-6 {
					t.Fatalf("iter %d delta=%v: after push-out %+v", iter, delta, r)
				}
			}
		}
	}
}

func sub(p, q geometry.Point) geometry.Point {
	return geometry.Point{X: p.X - q.X, Y: p.Y - q.Y}
}
func normalize(p geometry.Point) geometry.Point {
	l := vlen(p)
	if l == 0 {
		return geometry.Point{}
	}
	return geometry.Point{X: p.X / l, Y: p.Y / l}
}

type analytic struct {
	separated bool
	value     float64
	nx, ny    float64
}

// analyticRect 是轴对齐矩形间隙/穿透的精确解析解，方向定义为推开 B。
func analyticRect(acx, acy, ahw, ahh, bcx, bcy, bhw, bhh float64) analytic {
	aminx, amaxx := acx-ahw, acx+ahw
	bminx, bmaxx := bcx-bhw, bcx+bhw
	aminy, amaxy := acy-ahh, acy+ahh
	bminy, bmaxy := bcy-bhh, bcy+bhh

	// 每轴：>0 表示 B 在 A 正侧的间隙；<0 表示需要向 +x 推开的重叠；
	// 另外记录“向负侧”的对应量。
	xGapPos := bminx - amaxx
	xGapNeg := aminx - bmaxx
	yGapPos := bminy - amaxy
	yGapNeg := aminy - bmaxy

	var dx, dy float64 // 有符号分离向量分量（推开 B 的方向）
	xSep, ySep := false, false
	switch {
	case xGapPos >= 0:
		dx, xSep = xGapPos, true
	case xGapNeg >= 0:
		dx, xSep = -xGapNeg, true
	default:
		// 重叠：记录两个候选推出量。
	}
	switch {
	case yGapPos >= 0:
		dy, ySep = yGapPos, true
	case yGapNeg >= 0:
		dy, ySep = -yGapNeg, true
	}

	if xSep || ySep {
		if xSep && ySep {
			d := math.Hypot(dx, dy)
			return analytic{true, d, dx / d, dy / d}
		}
		if xSep {
			return analytic{true, math.Abs(dx), math.Copysign(1, dx), 0}
		}
		return analytic{true, math.Abs(dy), 0, math.Copysign(1, dy)}
	}

	// 两轴都穿透：MTD 取较小重叠。
	overlapXpos := amaxx - bminx // 把 B 推向 +x
	overlapXneg := bmaxx - aminx // 把 B 推向 -x
	overlapYpos := amaxy - bminy
	overlapYneg := bmaxy - aminy
	depthX := math.Min(overlapXpos, overlapXneg)
	depthY := math.Min(overlapYpos, overlapYneg)
	if depthX <= depthY {
		s := 1.0
		if overlapXneg < overlapXpos {
			s = -1
		}
		return analytic{false, depthX, s, 0}
	}
	s := 1.0
	if overlapYneg < overlapYpos {
		s = -1
	}
	return analytic{false, depthY, 0, s}
}

func inRect(p geometry.Point, cx, cy, hw, hh float64) bool {
	const e = 1e-7
	return p.X >= cx-hw-e && p.X <= cx+hw+e && p.Y >= cy-hh-e && p.Y <= cy+hh+e
}
