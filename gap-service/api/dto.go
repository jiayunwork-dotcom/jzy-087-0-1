package api

import (
	"encoding/json"
	"fmt"
	"math"

	"github.com/assembly-gap/gap-service/geometry"
)

// PointDTO 接收顶点，同时支持对象 {"x":1.0,"y":2.0} 与数组 [1.0,2.0] 两种写法。
type PointDTO struct {
	X, Y float64
}

func (p *PointDTO) UnmarshalJSON(data []byte) error {
	// 数组形式 [x, y]
	var arr []json.Number
	if err := json.Unmarshal(data, &arr); err == nil {
		if len(arr) != 2 {
			return fmt.Errorf("vertex as array must have exactly 2 elements, got %d", len(arr))
		}
		x, errX := arr[0].Float64()
		y, errY := arr[1].Float64()
		if errX != nil || errY != nil {
			return fmt.Errorf("vertex coordinates must be numbers")
		}
		p.X, p.Y = x, y
		return nil
	}

	// 对象形式 {"x":..,"y":..}
	var obj struct {
		X json.Number `json:"x"`
		Y json.Number `json:"y"`
	}
	if err := json.Unmarshal(data, &obj); err != nil {
		return fmt.Errorf("vertex must be [x, y] or {\"x\":..,\"y\":..}")
	}
	if obj.X == "" || obj.Y == "" {
		return fmt.Errorf("vertex must contain both x and y")
	}
	x, errX := obj.X.Float64()
	y, errY := obj.Y.Float64()
	if errX != nil || errY != nil {
		return fmt.Errorf("vertex coordinates must be numbers")
	}
	p.X, p.Y = x, y
	return nil
}

// QueryRequest 是 POST /query 的请求体。
type QueryRequest struct {
	PolygonA []PointDTO `json:"polygon_a"`
	PolygonB []PointDTO `json:"polygon_b"`
}

// PointView 是出参中的点。
type PointView struct {
	X float64 `json:"x"`
	Y float64 `json:"y"`
}

func toView(p geometry.Point) PointView {
	v := PointView{X: p.X, Y: p.Y}
	// 规整 -0，避免响应里出现 "y":-0 这种对人不友好的写法。
	if v.X == 0 {
		v.X = 0
	}
	if v.Y == 0 {
		v.Y = 0
	}
	return v
}
func toPoints(dtos []PointDTO) []geometry.Point {
	out := make([]geometry.Point, len(dtos))
	for i, d := range dtos {
		out[i] = geometry.Point{X: d.X, Y: d.Y}
	}
	return out
}

// QueryResponse 是核算成功时的响应。Distance/Depth 按分支仅填充其一。
type QueryResponse struct {
	Status   string    `json:"status"`
	Distance *float64  `json:"distance,omitempty"`
	Depth    *float64  `json:"depth,omitempty"`
	Normal   PointView `json:"normal"`
	PointOnA PointView `json:"point_on_a"`
	PointOnB PointView `json:"point_on_b"`
	Steps    int       `json:"iterations"`
}

// ErrorResponse 是所有错误的统一格式：机器可读代码 + 人类可读说明。
type ErrorResponse struct {
	Code    string `json:"code"`
	Message string `json:"message"`
}

func finite(polys ...[]geometry.Point) bool {
	for _, poly := range polys {
		for _, p := range poly {
			if math.IsNaN(p.X) || math.IsNaN(p.Y) || math.IsInf(p.X, 0) || math.IsInf(p.Y, 0) {
				return false
			}
		}
	}
	return true
}
