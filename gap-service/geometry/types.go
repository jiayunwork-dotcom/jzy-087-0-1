// Package geometry 实现两个平面凸多边形之间的间隙 / 穿透核算。
//
// 核心方法是 Minkowski 差 A⊖B 上的 GJK 单纯形迭代：
//   - 原点不在 A⊖B 内  -> 两体分离，距离为原点到差集边界的距离；
//   - 原点落在 A⊖B 内  -> 两体穿透，由 EPA（Expanding Polytope Algorithm，
//     平面上即多面体扩展）求原点到差集最近边，边长即穿透深度，边的外法向即
//     将 B 相对 A 推开的方向。
package geometry

import "errors"

// Point 是平面顶点。
type Point struct {
	X float64 `json:"x"`
	Y float64 `json:"y"`
}

// MinkowskiPoint 是 Minkowski 差上的一个单纯形顶点，
// 同时保留来自 A、B 两个多边形上的支撑顶点，用于还原最近点对。
type MinkowskiPoint struct {
	V    Point // a - b，差集上的点
	A, B Point // 分别来自 A、B 的支撑点
}

// 迭代相关的错误。两分支（GJK 距离迭代、EPA 多面体扩展）均设有硬性迭代上限，
// 用尽仍未收敛时返回错误，绝不退化成“距离为零却声称分离”。
var (
	ErrGJKNotConverged = errors.New("GJK simplex iteration did not converge within the iteration limit")
	ErrEPANotConverged = errors.New("EPA polytope expansion did not converge within the iteration limit")
)

// 几何判据的基础容差。涉及尺度的判据会额外引入相对容差，
// 见 polygonScale 与各自算法中的说明。
const (
	EpsAbs = 1e-9
)

// 两分支的迭代上限。默认 64，包内测试可临时改小以验证“迭代用尽即报错”。
var (
	MaxGJKSteps = 64
	MaxEPASteps = 64
)
