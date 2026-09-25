# gap-service — 平面凸多边形装配间隙 / 穿透核算服务

两个零件轮廓被简化为平面凸多边形。服务接收两多边形顶点，回答：

- **分离 (`separated`)**：最近距离、最近点对、接触法向；
- **穿透 (`penetrating`)**：穿透深度、把两者恰好推开的方向（MTD，最小平移距离）。

算法内核是 **Minkowski 差 A⊖B 上的 GJK 单纯形迭代**：

1. 以支撑函数沿给定方向取两多边形投影极值，构造差集上的支撑点；
2. 单纯形迭代判定原点是否被包住 —— 包住即穿透，否则分离；
3. 分离分支（距离版 GJK）用原点到当前单纯形的最近特征收敛出真实最近距离，
   并用支撑顶点的重心系数还原两多边形上的最近点对；
4. 穿透分支用 **EPA（多面体扩展，平面上即凸多边形扩张）**：从包围原点的
   初始单纯形出发，反复沿最近边外法向取支撑点扩大多面体，直到支撑点不再
   越过最近边；该边到原点的距离即穿透深度，边的外法向即分离方向。

不使用包围盒中心距等任何近似；凹多边形直接拒绝，不做凸分解或凸包化。

## 模块划分

```
geometry/
  types.go    点 / Minkowski 点 / 容差与迭代上限
  support.go  支撑函数（单体支撑、Minkowski 差支撑）
  simplex.go  单纯形上演化：最近点、重心系数、含原点判定
  gjk.go      GJK 距离迭代 + 含原点判定 + 分离分支最近距离/最近点
  epa.go      穿透分支 EPA 多面体扩展（最近边、扩张、接触点）
  seed.go     EPA 初始凸包（含零距离接触的退化种子处理）
  validate.go 顶点数 / 共线退化 / 零面积 / 凸性校验
api/
  dto.go      请求响应结构、顶点两种 JSON 写法（数组 / 对象）
  handler.go  Gin 接入与错误码映射
main.go       HTTP 服务入口（含容器健康检查自检模式）
```

## 运行

### 容器（一条命令）

```bash
docker compose up --build -d
# 接口： http://localhost:8080/query
```

或直接用 Docker：

```bash
docker build -t assembly-gap-service .
docker run --rm -p 8080:8080 assembly-gap-service
```

监听地址可用环境变量 `GAP_SERVICE_ADDR` 覆盖（默认 `:8080`）。

### 本地

```bash
go run .
```

## 接口

### `POST /query`

请求体（顶点支持 `[x, y]` 或 `{"x":..,"y":..}`，环绕顺序顺/逆时针均可，
允许边上有共线冗余顶点）：

```json
{
  "polygon_a": [[-4, -1], [-1, -1], [-1, 1], [-4, 1]],
  "polygon_b": [[2, -1], [4, -1], [4, 1], [2, 1]]
}
```

分离响应：

```json
{
  "status": "separated",
  "distance": 3,
  "normal": {"x": 1, "y": 0},
  "point_on_a": {"x": -1, "y": -1},
  "point_on_b": {"x": 2, "y": -1},
  "iterations": 1
}
```

穿透响应（`depth` 为穿透深度，`normal` 为把 B 从 A 上推开的单位方向）：

```json
{
  "status": "penetrating",
  "depth": 0.3,
  "normal": {"x": 1, "y": 0},
  "point_on_a": {"x": 1, "y": 0},
  "point_on_b": {"x": 0.7, "y": 0},
  "iterations": 4
}
```

法向约定：**统一为把 `polygon_b` 推离 `polygon_a` 的单位向量**。
恰好接触（零距离）归为 `penetrating` 且 `depth=0`，不会报成「距离为零的分离」。

错误响应统一为 `{"code": ..., "message": ...}`：

| HTTP | code | 含义 |
| --- | --- | --- |
| 400 | `INVALID_JSON` / `MISSING_POLYGON` / `NON_FINITE_COORDINATE` | 请求格式问题 |
| 422 | `TOO_FEW_VERTICES` | 任一多边形顶点少于 3 |
| 422 | `DEGENERATE_POLYGON` | 相邻顶点重合 |
| 422 | `ZERO_AREA_POLYGON` | 面积退化为零（共线等） |
| 422 | `NON_CONVEX_POLYGON` | 非凸，直接拒绝，不做凸分解 |
| 422 | `GJK_NO_CONVERGENCE` / `EPA_NO_CONVERGENCE` | 迭代上限用尽（默认各 64 次） |

### `GET /health`

返回 `{"status":"ok"}`。

## 随服务附带的已知算例

`examples/separated_gap3.json`：两个轴对齐矩形，A 右边 `x=-1`，B 左边 `x=2`，
边到边缝隙 **3**，接触法向 **(1, 0)**。

```bash
curl -s -X POST localhost:8080/query \
  -H 'Content-Type: application/json' \
  --data @examples/separated_gap3.json
```

`examples/penetration_depth03.json`：x 向重叠 0.3、y 向重叠 2，
穿透深度应对上较小重叠量 **0.3**，法向 **(1, 0)**。

## 数值容差与收敛

- 内部基础绝对容差 `1e-9`，并按特征尺度叠加相对容差（绝对+相对）；
- GJK 收敛判据：新支撑点越过当前最近点支撑平面的推进量落入容差；
- EPA 收敛判据：最近边支撑点在边外法向上不再显著越过该边；
- GJK / EPA 各设硬性迭代上限 64（`geometry.MaxGJKSteps` /
  `geometry.MaxEPASteps`），用尽返回错误而非死循环或错报零距离。

## 测试

```bash
go test ./...
```

覆盖内容（断言均带显式容差，非「有返回值即可」）：

- 顶点不足 / 共线零面积 / 重合顶点等退化与非法输入；
- 凹多边形拒绝，且 `Query` 层面同样拒绝（无凸包化）；
- 轴对齐矩形缝隙精确等于边到边间隔，法向与最近点坐标逐项核对；
- 沿接触法向平移一个零件，距离随位移等量增 / 减；
- 两零件同时做同一平移，间隙与法向保持不变（穿透分支同样成立）；
- 重叠矩形深度等于较小重叠尺寸；沿法向平移 depth+δ 后间隙恰为 δ；
- 零距离接触报穿透（depth=0），包含关系、角点-角点、旋转不变性、
  顺/逆时针环绕；
- 400 组随机轴对齐矩形对解析解、600 组随机凸多边形对
  SAT（穿透深度/方向）与暴力点-边距离枚举的交叉验证；
- GJK / EPA 迭代上限用尽必须返回错误；
- HTTP 层成功与全部错误码、健康检查。
