package api_test

import (
	"bytes"
	"encoding/json"
	"net/http"
	"net/http/httptest"
	"testing"

	"github.com/gin-gonic/gin"

	"github.com/assembly-gap/gap-service/api"
)

func setupRouter() *gin.Engine {
	gin.SetMode(gin.TestMode)
	r := gin.New()
	api.Register(r)
	return r
}

func postJSON(t *testing.T, r *gin.Engine, body string) (int, map[string]any) {
	t.Helper()
	req := httptest.NewRequest(http.MethodPost, "/query", bytes.NewBufferString(body))
	req.Header.Set("Content-Type", "application/json")
	w := httptest.NewRecorder()
	r.ServeHTTP(w, req)

	var out map[string]any
	if w.Body.Len() > 0 {
		if err := json.Unmarshal(w.Body.Bytes(), &out); err != nil {
			t.Fatalf("invalid JSON response: %v: %s", err, w.Body.String())
		}
	}
	return w.Code, out
}

func TestHTTP_Separated_KnownGap(t *testing.T) {
	r := setupRouter()
	body := `{
	  "polygon_a": [[-4,-1],[-1,-1],[-1,1],[-4,1]],
	  "polygon_b": [[2,-1],[4,-1],[4,1],[2,1]]
	}`
	code, out := postJSON(t, r, body)
	if code != http.StatusOK {
		t.Fatalf("status=%d body=%v", code, out)
	}
	if out["status"] != "separated" {
		t.Fatalf("status=%v", out["status"])
	}
	if d, _ := out["distance"].(float64); d != 3.0 {
		t.Fatalf("distance=%v want 3", out["distance"])
	}
	if _, ok := out["depth"]; ok {
		t.Fatalf("separated response must not carry depth: %v", out["depth"])
	}
	n := out["normal"].(map[string]any)
	if n["x"].(float64) != 1 || n["y"].(float64) != 0 {
		t.Fatalf("normal=%v want (1,0)", n)
	}
}

func TestHTTP_Penetration_Depth(t *testing.T) {
	r := setupRouter()
	// A: x∈[-1,1] y∈[-1.5,1.5]; B: x∈[0.7,2.7] y∈[-1,1] => x 重叠 0.3
	body := `{
	  "polygon_a": {"vertices":[[-1,-1.5],[1,-1.5],[1,1.5],[-1,1.5]]},
	  "polygon_b": [[0.7,-1],[2.7,-1],[2.7,1],[0.7,1]]
	}`
	// polygon_a 传成对象（数组才是合法格式）-> 绑定阶段即拒绝，不允许静默忽略。
	code, out := postJSON(t, r, body)
	if code != http.StatusBadRequest || out["code"] != "INVALID_JSON" {
		t.Fatalf("want 400 INVALID_JSON, got %d %v", code, out)
	}

	body = `{
	  "polygon_a": [[-1,-1.5],[1,-1.5],[1,1.5],[-1,1.5]],
	  "polygon_b": [[0.7,-1],[2.7,-1],[2.7,1],[0.7,1]]
	}`
	code, out = postJSON(t, r, body)
	if code != http.StatusOK {
		t.Fatalf("status=%d body=%v", code, out)
	}
	if out["status"] != "penetrating" {
		t.Fatalf("status=%v", out["status"])
	}
	d, _ := out["depth"].(float64)
	if d-0.3 > 1e-9 || 0.3-d > 1e-9 {
		t.Fatalf("depth=%v want 0.3", out["depth"])
	}
	if _, ok := out["distance"]; ok {
		t.Fatalf("penetrating response must not carry distance")
	}
}

func TestHTTP_Errors(t *testing.T) {
	r := setupRouter()

	cases := []struct {
		name     string
		body     string
		wantCode int
		wantErr  string
	}{
		{
			name:     "invalid json",
			body:     `{not json`,
			wantCode: http.StatusBadRequest,
			wantErr:  "INVALID_JSON",
		},
		{
			name:     "missing polygon",
			body:     `{"polygon_a":[[0,0],[1,0],[1,1]]}`,
			wantCode: http.StatusBadRequest,
			wantErr:  "MISSING_POLYGON",
		},
		{
			name:     "too few vertices",
			body:     `{"polygon_a":[[0,0],[1,0]],"polygon_b":[[0,0],[1,0],[1,1]]}`,
			wantCode: http.StatusUnprocessableEntity,
			wantErr:  "TOO_FEW_VERTICES",
		},
		{
			name:     "zero area",
			body:     `{"polygon_a":[[0,0],[1,0],[2,0]],"polygon_b":[[0,0],[1,0],[1,1]]}`,
			wantCode: http.StatusUnprocessableEntity,
			wantErr:  "ZERO_AREA_POLYGON",
		},
		{
			name: "non convex",
			body: `{"polygon_a":[[0,0],[2,0],[2,1],[1,1],[1,2],[0,2]],` +
				`"polygon_b":[[5,5],[6,5],[6,6],[5,6]]}`,
			wantCode: http.StatusUnprocessableEntity,
			wantErr:  "NON_CONVEX_POLYGON",
		},
		{
			name:     "bad vertex shape",
			body:     `{"polygon_a":[[0,0],[1,0],[1]],"polygon_b":[[0,0],[1,0],[1,1]]}`,
			wantCode: http.StatusBadRequest,
			wantErr:  "INVALID_JSON",
		},
	}

	for _, tc := range cases {
		t.Run(tc.name, func(t *testing.T) {
			code, out := postJSON(t, r, tc.body)
			if code != tc.wantCode {
				t.Fatalf("code=%d want %d, body=%v", code, tc.wantCode, out)
			}
			if out["code"] != tc.wantErr {
				t.Fatalf("error code=%v want %s; message=%v", out["code"], tc.wantErr, out["message"])
			}
			if msg, _ := out["message"].(string); msg == "" {
				t.Fatalf("error message must be human-readable, got %v", out)
			}
		})
	}
}

func TestHTTP_Health(t *testing.T) {
	r := setupRouter()
	req := httptest.NewRequest(http.MethodGet, "/health", nil)
	w := httptest.NewRecorder()
	r.ServeHTTP(w, req)
	if w.Code != http.StatusOK {
		t.Fatalf("health code=%d", w.Code)
	}
}
