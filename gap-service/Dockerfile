# syntax=docker/dockerfile:1

# ---- 构建阶段 ----
FROM golang:1.22-bookworm AS build

WORKDIR /src

# 先只拷贝依赖清单，利用层缓存下载模块。
COPY go.mod go.sum ./
RUN --mount=type=cache,target=/go/pkg/mod \
    go mod download

COPY . .

# 纯静态构建（CGO_ENABLED=0），可直接跑在 scratch/distroless 上。
RUN --mount=type=cache,target=/go/pkg/mod \
    --mount=type=cache,target=/root/.cache/go-build \
    CGO_ENABLED=0 GOOS=linux go build \
    -trimpath -ldflags="-s -w" -o /out/gap-service .

# ---- 运行阶段：最小镜像，无 shell、无包管理器 ----
FROM gcr.io/distroless/static-debian12:nonroot

WORKDIR /
COPY --from=build /out/gap-service /gap-service

EXPOSE 8080
USER nonroot:nonroot
ENTRYPOINT ["/gap-service"]
