// Command gap-service 启动凸多边形间隙 / 穿透核算 HTTP 服务。
package main

import (
	"flag"
	"log"
	"net/http"
	"os"
	"time"

	"github.com/gin-gonic/gin"

	"github.com/assembly-gap/gap-service/api"
)

func main() {
	// -check URL：自检模式（供容器健康检查使用），对给定地址发一次 GET。
	check := flag.String("check", "", "if set, perform a one-shot GET health check against this URL and exit")
	flag.Parse()
	if *check != "" {
		os.Exit(healthCheck(*check))
	}

	gin.SetMode(gin.ReleaseMode)
	r := gin.New()
	r.Use(gin.Recovery())
	api.Register(r)

	addr := os.Getenv("GAP_SERVICE_ADDR")
	if addr == "" {
		addr = ":8080"
	}
	log.Printf("gap-service listening on %s", addr)
	if err := r.Run(addr); err != nil {
		log.Fatalf("server failed: %v", err)
	}
}

func healthCheck(url string) int {
	client := http.Client{Timeout: 2 * time.Second}
	resp, err := client.Get(url)
	if err != nil {
		log.Printf("health check failed: %v", err)
		return 1
	}
	defer resp.Body.Close()
	if resp.StatusCode != http.StatusOK {
		log.Printf("health check: status %d", resp.StatusCode)
		return 1
	}
	return 0
}
