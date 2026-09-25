package api

import (
	"errors"
	"net/http"

	"github.com/gin-gonic/gin"

	"github.com/assembly-gap/gap-service/geometry"
)

// Register 把查询接口挂载到给定 Gin 引擎上。
func Register(r *gin.Engine) {
	r.GET("/health", func(c *gin.Context) { c.JSON(http.StatusOK, gin.H{"status": "ok"}) })
	r.POST("/query", handleQuery)
}

// handleQuery 是唯一的对外几何核算入口。
func handleQuery(c *gin.Context) {
	var req QueryRequest
	if err := c.ShouldBindJSON(&req); err != nil {
		c.JSON(http.StatusBadRequest, ErrorResponse{
			Code:    "INVALID_JSON",
			Message: "request body must be JSON with polygon_a and polygon_b: " + err.Error(),
		})
		return
	}
	if len(req.PolygonA) == 0 || len(req.PolygonB) == 0 {
		c.JSON(http.StatusBadRequest, ErrorResponse{
			Code:    "MISSING_POLYGON",
			Message: "both polygon_a and polygon_b are required and must be non-empty",
		})
		return
	}

	a := toPoints(req.PolygonA)
	b := toPoints(req.PolygonB)
	if !finite(a, b) {
		c.JSON(http.StatusBadRequest, ErrorResponse{
			Code:    "NON_FINITE_COORDINATE",
			Message: "vertex coordinates must be finite numbers",
		})
		return
	}

	if err := validateBoth(a, b); err != nil {
		writeValidationError(c, err)
		return
	}

	res, err := geometry.Query(a, b)
	if err != nil {
		writeComputeError(c, err)
		return
	}

	resp := QueryResponse{
		Status:   string(res.Status),
		Normal:   toView(res.Normal),
		PointOnA: toView(res.PointOnA),
		PointOnB: toView(res.PointOnB),
		Steps:    res.Steps,
	}
	switch res.Status {
	case geometry.StatusSeparated:
		d := res.Distance
		resp.Distance = &d
	case geometry.StatusPenetrating:
		d := res.Depth
		resp.Depth = &d
	}
	c.JSON(http.StatusOK, resp)
}

func validateBoth(a, b []geometry.Point) error {
	if err := geometry.ValidatePolygon(a); err != nil {
		return err
	}
	return geometry.ValidatePolygon(b)
}

func writeValidationError(c *gin.Context, err error) {
	type mapping struct {
		target error
		code   string
	}
	mappings := []mapping{
		{geometry.ErrTooFewVertices, "TOO_FEW_VERTICES"},
		{geometry.ErrNonFinite, "NON_FINITE_COORDINATE"},
		{geometry.ErrDuplicatePoint, "DEGENERATE_POLYGON"},
		{geometry.ErrZeroArea, "ZERO_AREA_POLYGON"},
		{geometry.ErrNonConvex, "NON_CONVEX_POLYGON"},
	}
	for _, m := range mappings {
		if errors.Is(err, m.target) {
			c.JSON(http.StatusUnprocessableEntity, ErrorResponse{Code: m.code, Message: err.Error()})
			return
		}
	}
	c.JSON(http.StatusUnprocessableEntity, ErrorResponse{Code: "INVALID_POLYGON", Message: err.Error()})
}

func writeComputeError(c *gin.Context, err error) {
	switch {
	case errors.Is(err, geometry.ErrGJKNotConverged):
		c.JSON(http.StatusUnprocessableEntity, ErrorResponse{
			Code:    "GJK_NO_CONVERGENCE",
			Message: err.Error(),
		})
	case errors.Is(err, geometry.ErrEPANotConverged):
		c.JSON(http.StatusUnprocessableEntity, ErrorResponse{
			Code:    "EPA_NO_CONVERGENCE",
			Message: err.Error(),
		})
	default:
		c.JSON(http.StatusInternalServerError, ErrorResponse{
			Code:    "COMPUTATION_FAILED",
			Message: err.Error(),
		})
	}
}
