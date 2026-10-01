"""
智慧物流路徑最佳化服務 (Multi-Depot Open-trip TSP + OSRM)
"""
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


# ================= 数据模型 =================
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


# ================= 核心演算法模組 =================

def fetch_osrm_table(coords: List[List[float]]) -> Dict[str, Any]:
    """
    呼叫 OSRM Table Service 取得完整 NxN 距離與時間矩陣
    coords 格式: [[lon1, lat1], [lon2, lat2], ...]
    """
    coords_str = ";".join([f"{c[0]:.6f},{c[1]:.6f}" for c in coords])
    url = f"{OSRM_BASE_URL}/table/v1/driving/{coords_str}?annotations=distance,duration"
    
    resp = requests.get(url, timeout=15)
    if resp.status_code != 200:
        raise HTTPException(status_code=502, detail=f"OSRM Table 請求失敗: {resp.text}")
    
    data = resp.json()
    if data.get("code") != "Ok":
        raise HTTPException(status_code=500, detail=f"OSRM 錯誤: {data.get('code')}")
    
    return data  # 包含 'distances' 與 'durations'


def open_tsp_2opt(node_indices: List[int], dist_matrix: List[List[float]], start_idx: int) -> List[int]:
    """
    開放式 TSP (起點固定為 start_idx，不回場)
    1. 最鄰近法 (Nearest Neighbor) 建立初始路徑
    2. 2-opt 局部搜尋消除交叉
    """
    unvisited = [n for n in node_indices if n != start_idx]
    route = [start_idx]

    # 1. 最近鄰法建立初始解
    curr = start_idx
    while unvisited:
        next_node = min(unvisited, key=lambda n: dist_matrix[curr][n])
        route.append(next_node)
        unvisited.remove(next_node)
        curr = next_node

    # 2. 開放式 2-opt 局部優化 (起點 route[0] 保持不動)
    improved = True
    n_nodes = len(route)
    
    def calculate_open_route_cost(r):
        return sum(dist_matrix[r[k]][r[k+1]] for k in range(len(r) - 1))

    best_cost = calculate_open_route_cost(route)

    while improved:
        improved = False
        for i in range(1, n_nodes - 1):
            for j in range(i + 1, n_nodes):
                # 反轉 i 到 j 的子路徑
                new_route = route[:i] + route[i:j+1][::-1] + route[j+1:]
                new_cost = calculate_open_route_cost(new_route)
                if new_cost < best_cost - 1e-4:
                    route = new_route
                    best_cost = new_cost
                    improved = True
                    break
            if improved:
                break

    return route


def get_osrm_geometry(ordered_coords: List[List[float]]) -> Dict[str, Any]:
    """
    呼叫 OSRM Route Service 取得實際道路 Polyline 軌跡與詳細指標
    ordered_coords: [[lon, lat], ...]
    """
    if len(ordered_coords) < 2:
        return {"geometry": [], "distance": 0, "duration": 0}

    coords_str = ";".join([f"{c[0]:.6f},{c[1]:.6f}" for c in ordered_coords])
    url = f"{OSRM_BASE_URL}/route/v1/driving/{coords_str}?overview=full&geometries=geojson"
    
    resp = requests.get(url, timeout=10)
    if resp.status_code == 200:
        data = resp.json()
        if data.get("routes"):
            route_info = data["routes"][0]
            # GeoJSON 格式為 [lon, lat]，轉回 Leaflet 慣用的 [lat, lon]
            latlng_path = [[pt[1], pt[0]] for pt in route_info["geometry"]["coordinates"]]
            return {
                "geometry": latlng_path,
                "distance_km": round(route_info["distance"] / 1000.0, 2),
                "duration_min": round(route_info["duration"] / 60.0, 1)
            }

    return {"geometry": [], "distance_km": 0, "duration_min": 0}


# ================= 主排程入口 API =================

@app.post("/api/optimize-route")
def optimize_routes(payload: OptimizeRequest):
    if not payload.depots:
        raise HTTPException(status_code=400, detail="至少需要提供一個出發點 (Depot)")
    if not payload.stops:
        raise HTTPException(status_code=400, detail="未提供任何配送點位")

    # 1. 單號打包 (Order Aggregation)：同單號的點位包裝為一個整體行程節點
    orders_map: Dict[str, List[DeliveryStop]] = {}
    for stop in payload.stops:
        orders_map.setdefault(stop.order_id, []).append(stop)

    # 2. 建立全域座標表以呼叫 OSRM Table
    # 順序：[Depot_0, Depot_1, ..., Order_Rep_0, Order_Rep_1, ...]
    all_coords = []
    depot_indices = []
    for d in payload.depots:
        depot_indices.append(len(all_coords))
        all_coords.append([d.lon, d.lat])

    order_ids = list(orders_map.keys())
    order_entry_indices = {}  # order_id -> 在 Table 中的 index (以該單號第 1 個站點為代表)
    for oid in order_ids:
        entry_stop = orders_map[oid][0]
        order_entry_indices[oid] = len(all_coords)
        all_coords.append([entry_stop.lon, entry_stop.lat])

    # 3. 呼叫 OSRM 獲取各點行車距離矩陣
    table_result = fetch_osrm_table(all_coords)
    dist_matrix = table_result["distances"]  # 單位：公尺

    # 4. 多出發點最適指派 (Multi-Depot Assignment)
    # 比對所有出發點至該單號第一站的距離，將單號分派給代價最低的出發點
    depot_assigned_orders: Dict[int, List[str]] = {d_idx: [] for d_idx in depot_indices}

    for oid in order_ids:
        order_mat_idx = order_entry_indices[oid]
        # 尋找與該單號最近的出發點
        best_depot = min(depot_indices, key=lambda d_idx: dist_matrix[d_idx][order_mat_idx])
        depot_assigned_orders[best_depot].append(oid)

    # 5. 針對各出發點所屬單號群執行開放式 TSP 排序 + 2-opt
    routes_summary = []

    for d_idx, assigned_oids in depot_assigned_orders.items():
        if not assigned_oids:
            continue

        assigned_depot = payload.depots[d_idx]

        # 構建局部子矩陣節點 (起點 + 所屬單號)
        sub_node_ids = [d_idx] + [order_entry_indices[oid] for oid in assigned_oids]
        
        # 開放式 TSP 排序 (不回場)
        ordered_indices = open_tsp_2opt(sub_node_ids, dist_matrix, start_idx=d_idx)

        # 展開回真實送貨節點清單 (同單號內部點位連續送完)
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
                # 展開該單號底下的全部停靠站
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

        # 6. 呼叫 OSRM Route 取得實際路網幾何線條
        route_geo = get_osrm_geometry(ordered_geo_coords)

        routes_summary.append({
            "depot_id": assigned_depot.id,
            "depot_name": assigned_depot.name,
            "total_stops": len(final_stops_sequence) - 1,
            "total_distance_km": route_geo["distance_km"],
            "estimated_duration_min": route_geo["duration_min"],
            "sequence": final_stops_sequence,
            "polyline": route_geo["geometry"]  # [[lat, lon], ...]
        })

    return {
        "status": "success",
        "routes": routes_summary
    }

if __name__ == "__main__":
    import uvicorn
    # 本地啟動 API 伺服器
    uvicorn.run(app, host="127.0.0.1", port=8000)