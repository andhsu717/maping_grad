"""
智慧物流路徑最佳化服務 (Multi-Depot Open-trip TSP + OSRM)
修正版：強化 OSRM 連線穩定度、異常捕獲與備用距離機制
"""
import math
import requests
from typing import List, Dict, Any, Optional
from fastapi import FastAPI, HTTPException
from fastapi.middleware.cors import CORSMiddleware
from pydantic import BaseModel

app = FastAPI(title="Logistics Route Optimizer API")

# 允許跨來源請求 (供前端網頁直接呼叫)
app.add_middleware(
    CORSMiddleware,
    allow_origins=["*"],
    allow_credentials=True,
    allow_methods=["*"],
    allow_headers=["*"],
)

OSRM_BASE_URL = "https://router.project-osrm.org"
HEADERS = {
    "User-Agent": "SmartLogisticsApp/1.0 (Python-FastAPI-Client)",
    "Accept": "application/json"
}


# ================= 資料模型 =================
class DepotItem(BaseModel):
    id: str
    name: str
    lat: float
    lon: float

class DeliveryStop(BaseModel):
    order_id: str
    stop_id: str
    name: Optional[str] = ""
    address: Optional[str] = ""
    lat: float
    lon: float

class OptimizeRequest(BaseModel):
    depots: List[DepotItem]
    stops: List[DeliveryStop]


# ================= 備用距離運算 (Haversine 大圓距離) =================

def compute_haversine_matrix(coords: List[List[float]]) -> List[List[float]]:
    """
    當 OSRM 超過上限或無法連線時使用的備用距離矩陣 (單位：公尺)
    coords: [[lon, lat], ...]
    """
    R = 6371000  # 地球半徑 (公尺)
    n = len(coords)
    matrix = [[0.0] * n for _ in range(n)]

    for i in range(n):
        lon1, lat1 = math.radians(coords[i][0]), math.radians(coords[i][1])
        for j in range(i + 1, n):
            lon2, lat2 = math.radians(coords[j][0]), math.radians(coords[j][1])
            dlat = lat2 - lat1
            dlon = lon2 - lon1
            a = math.sin(dlat / 2)**2 + math.cos(lat1) * math.cos(lat2) * math.sin(dlon / 2)**2
            c = 2 * math.atan2(math.sqrt(a), math.sqrt(1 - a))
            dist = R * c
            matrix[i][j] = dist
            matrix[j][i] = dist
    return matrix


# ================= OSRM API 串接模組 =================

def fetch_osrm_table(coords: List[List[float]]) -> List[List[float]]:
    """
    呼叫 OSRM Table Service 取得完整 NxN 距離矩陣
    若點位超過 100 點或 OSRM 失敗，自動平滑退回至地理距離計算
    """
    if len(coords) > 100:
        print(f"[警告] 點位數 ({len(coords)}) 超過 OSRM 公開伺服器限制 (100)，切換為幾何距離矩陣。")
        return compute_haversine_matrix(coords)

    coords_str = ";".join([f"{float(c[0]):.6f},{float(c[1]):.6f}" for c in coords])
    url = f"{OSRM_BASE_URL}/table/v1/driving/{coords_str}?annotations=distance"

    try:
        resp = requests.get(url, headers=HEADERS, timeout=12)
        if resp.status_code == 200:
            data = resp.json()
            if data.get("code") == "Ok" and "distances" in data:
                return data["distances"]
            else:
                print(f"[OSRM 回應異常] {data.get('code')}，切換至備用距離。")
        else:
            print(f"[OSRM 伺服器狀態碼: {resp.status_code}] 內容: {resp.text}")
    except requests.exceptions.RequestException as e:
        print(f"[OSRM Table 連線失敗] 錯誤原因: {e}，改用內建距離計算。")

    return compute_haversine_matrix(coords)


def get_osrm_geometry(ordered_coords: List[List[float]]) -> Dict[str, Any]:
    """
    呼叫 OSRM Route Service 取得實際道路軌跡 (GeoJSON Polyline)
    """
    if len(ordered_coords) < 2:
        return {"geometry": [], "distance_km": 0, "duration_min": 0}

    # 超過 80 個導航點時做步長抽樣，防止 Route API 網址過長或伺服器超載
    if len(ordered_coords) > 80:
        step = max(1, len(ordered_coords) // 80)
        sampled = ordered_coords[::step]
        if ordered_coords[-1] not in sampled:
            sampled.append(ordered_coords[-1])
    else:
        sampled = ordered_coords

    coords_str = ";".join([f"{float(c[0]):.6f},{float(c[1]):.6f}" for c in sampled])
    url = f"{OSRM_BASE_URL}/route/v1/driving/{coords_str}?overview=full&geometries=geojson"

    try:
        resp = requests.get(url, headers=HEADERS, timeout=10)
        if resp.status_code == 200:
            data = resp.json()
            if data.get("routes"):
                route_info = data["routes"][0]
                latlng_path = [[pt[1], pt[0]] for pt in route_info["geometry"]["coordinates"]]
                return {
                    "geometry": latlng_path,
                    "distance_km": round(route_info["distance"] / 1000.0, 2),
                    "duration_min": round(route_info["duration"] / 60.0, 1)
                }
    except Exception as e:
        print(f"[OSRM Route 軌跡獲取警告] {e}")

    # 若 Route API 失敗或逾時，退回以直線點連接，維持前端正常顯示
    fallback_path = [[c[1], c[0]] for c in ordered_coords]
    rough_dist = sum(
        math.hypot(ordered_coords[k][0] - ordered_coords[k+1][0], ordered_coords[k][1] - ordered_coords[k+1][1]) * 111
        for k in range(len(ordered_coords) - 1)
    )
    return {
        "geometry": fallback_path,
        "distance_km": round(rough_dist, 2),
        "duration_min": round(rough_dist / 35.0 * 60, 1)  # 粗估時速 35km/h
    }


# ================= 核心演算法：開放式 TSP (最鄰近法 + 2-opt) =================

def open_tsp_2opt(node_indices: List[int], dist_matrix: List[List[float]], start_idx: int) -> List[int]:
    """
    開放式 TSP (起點固定為 start_idx，不回場)
    1. 最鄰近法 (Nearest Neighbor)
    2. 2-opt 局部去交叉優化
    """
    unvisited = [n for n in node_indices if n != start_idx]
    route = [start_idx]

    # 1. 最近鄰法
    curr = start_idx
    while unvisited:
        next_node = min(unvisited, key=lambda n: dist_matrix[curr][n] if dist_matrix[curr][n] is not None else float('inf'))
        route.append(next_node)
        unvisited.remove(next_node)
        curr = next_node

    # 2. 開放式 2-opt 去交叉 (起點 route[0] 固定)
    improved = True
    n_nodes = len(route)

    def route_cost(r):
        return sum(dist_matrix[r[k]][r[k+1]] or 0 for k in range(len(r) - 1))

    best_cost = route_cost(route)
    iteration = 0

    while improved and iteration < 50:  # 限制迭代次數，保證毫秒級響應
        improved = False
        iteration += 1
        for i in range(1, n_nodes - 1):
            for j in range(i + 1, n_nodes):
                new_route = route[:i] + route[i:j+1][::-1] + route[j+1:]
                new_cost = route_cost(new_route)
                if new_cost < best_cost - 1e-3:
                    route = new_route
                    best_cost = new_cost
                    improved = True
                    break
            if improved:
                break

    return route


# ================= 主排程入口 API =================

@app.post("/api/optimize-route")
def optimize_routes(payload: OptimizeRequest):
    if not payload.depots:
        raise HTTPException(status_code=400, detail="請提供至少一個出發點 (Depot)")
    if not payload.stops:
        raise HTTPException(status_code=400, detail="未提供任何配送點位")

    # 1. 單號打包 (Order Aggregation)
    orders_map: Dict[str, List[DeliveryStop]] = {}
    for stop in payload.stops:
        if stop.lat and stop.lon:
            orders_map.setdefault(stop.order_id, []).append(stop)

    if not orders_map:
        raise HTTPException(status_code=400, detail="配送點中無有效的經緯度座標")

    # 2. 彙整全域座標
    all_coords = []
    depot_indices = []
    for d in payload.depots:
        depot_indices.append(len(all_coords))
        all_coords.append([float(d.lon), float(d.lat)])

    order_ids = list(orders_map.keys())
    order_entry_indices = {}
    for oid in order_ids:
        entry_stop = orders_map[oid][0]
        order_entry_indices[oid] = len(all_coords)
        all_coords.append([float(entry_stop.lon), float(entry_stop.lat)])

    # 3. 取得距離矩陣
    dist_matrix = fetch_osrm_table(all_coords)

    # 4. 多出發點指派 (為每個單號指派最近出發點)
    depot_assigned_orders: Dict[int, List[str]] = {d_idx: [] for d_idx in depot_indices}

    for oid in order_ids:
        order_mat_idx = order_entry_indices[oid]
        best_depot = min(
            depot_indices, 
            key=lambda d_idx: dist_matrix[d_idx][order_mat_idx] if dist_matrix[d_idx][order_mat_idx] is not None else float('inf')
        )
        depot_assigned_orders[best_depot].append(oid)

    # 5. 開放式 TSP 排序與展開路線
    routes_summary = []

    for d_idx, assigned_oids in depot_assigned_orders.items():
        if not assigned_oids:
            continue

        assigned_depot = payload.depots[d_idx]
        sub_node_ids = [d_idx] + [order_entry_indices[oid] for oid in assigned_oids]

        # 排序單號順序
        ordered_indices = open_tsp_2opt(sub_node_ids, dist_matrix, start_idx=d_idx)

        # 展開回站點順序清單 (同單號內部點位連續送完)
        final_stops_sequence = []
        ordered_geo_coords = [[assigned_depot.lon, assigned_depot.lat]]

        for idx in ordered_indices:
            if idx == d_idx:
                final_stops_sequence.append({
                    "type": "DEPOT",
                    "id": assigned_depot.id,
                    "name": assigned_depot.name,
                    "lat": assigned_depot.lat,
                    "lon": assigned_depot.lon
                })
            else:
                matched_oid = next(oid for oid, t_idx in order_entry_indices.items() if t_idx == idx)
                for stop in orders_map[matched_oid]:
                    final_stops_sequence.append({
                        "type": "STOP",
                        "order_id": stop.order_id,
                        "stop_id": stop.stop_id,
                        "name": stop.name,
                        "address": stop.address,
                        "lat": stop.lat,
                        "lon": stop.lon
                    })
                    ordered_geo_coords.append([stop.lon, stop.lat])

        # 取得道路幾何軌跡
        route_geo = get_osrm_geometry(ordered_geo_coords)

        routes_summary.append({
            "depot_id": assigned_depot.id,
            "depot_name": assigned_depot.name,
            "total_stops": len(final_stops_sequence) - 1,
            "total_distance_km": route_geo["distance_km"],
            "estimated_duration_min": route_geo["duration_min"],
            "sequence": final_stops_sequence,
            "polyline": route_geo["geometry"]
        })

    return {
        "status": "success",
        "routes": routes_summary
    }

if __name__ == "__main__":
    import uvicorn
    uvicorn.run(app, host="127.0.0.1", port=8000)