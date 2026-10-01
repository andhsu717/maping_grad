"""
智慧物流路徑最佳化服務 (Multi-Depot Open-trip TSP + OSRM)
更新：新增單行程優化 API (/api/optimize-single-trip)，強化佇列化排程與熱更新調度
"""
import math
import requests
from typing import List, Dict, Any, Optional
from fastapi import FastAPI, HTTPException
from fastapi.middleware.cors import CORSMiddleware
from pydantic import BaseModel

app = FastAPI(title="Logistics Route Optimizer API")

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
    stop_id: str
    order_id: Optional[str] = ""
    name: Optional[str] = ""
    address: Optional[str] = ""
    lat: float
    lon: float
    item_name: Optional[str] = ""
    weight_kg: Optional[float] = 0.0
    volume_cbm: Optional[float] = 0.0

class SingleTripRequest(BaseModel):
    order_id: str
    depot: DepotItem
    stops: List[DeliveryStop]

class OptimizeRequest(BaseModel):
    depots: List[DepotItem]
    stops: List[DeliveryStop]

# ================= 備用距離運算 (Haversine 大圓距離) =================
def compute_haversine_matrix(coords: List[List[float]]) -> List[List[float]]:
    R = 6371000  # 公尺
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

def fetch_osrm_table(coords: List[List[float]]) -> List[List[float]]:
    if len(coords) > 100:
        return compute_haversine_matrix(coords)
    coords_str = ";".join([f"{float(c[0]):.6f},{float(c[1]):.6f}" for c in coords])
    url = f"{OSRM_BASE_URL}/table/v1/driving/{coords_str}?annotations=distance"
    try:
        resp = requests.get(url, headers=HEADERS, timeout=12)
        if resp.status_code == 200:
            data = resp.json()
            if data.get("code") == "Ok" and "distances" in data:
                return data["distances"]
    except requests.exceptions.RequestException:
        pass
    return compute_haversine_matrix(coords)

def get_osrm_geometry(ordered_coords: List[List[float]]) -> Dict[str, Any]:
    if len(ordered_coords) < 2:
        return {"geometry": [], "distance_km": 0, "duration_min": 0}

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
    except Exception:
        pass

    fallback_path = [[c[1], c[0]] for c in ordered_coords]
    rough_dist = sum(
        math.hypot(ordered_coords[k][0] - ordered_coords[k+1][0], ordered_coords[k][1] - ordered_coords[k+1][1]) * 111
        for k in range(len(ordered_coords) - 1)
    )
    return {
        "geometry": fallback_path,
        "distance_km": round(rough_dist, 2),
        "duration_min": round(rough_dist / 35.0 * 60, 1)
    }

def open_tsp_2opt(node_indices: List[int], dist_matrix: List[List[float]], start_idx: int) -> List[int]:
    unvisited = [n for n in node_indices if n != start_idx]
    route = [start_idx]
    curr = start_idx
    while unvisited:
        next_node = min(unvisited, key=lambda n: dist_matrix[curr][n] if dist_matrix[curr][n] is not None else float('inf'))
        route.append(next_node)
        unvisited.remove(next_node)
        curr = next_node

    improved = True
    n_nodes = len(route)
    def route_cost(r):
        return sum(dist_matrix[r[k]][r[k+1]] or 0 for k in range(len(r) - 1))

    best_cost = route_cost(route)
    iteration = 0
    while improved and iteration < 50:
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

# ================= 新端點：單一行程最佳化 =================
@app.post("/api/optimize-single-trip")
def optimize_single_trip(payload: SingleTripRequest):
    valid_stops = [s for s in payload.stops if s.lat and s.lon]
    if not valid_stops:
        return {
            "status": "success",
            "order_id": payload.order_id,
            "total_stops": 0,
            "total_distance_km": 0,
            "estimated_duration_min": 0,
            "sequence": [],
            "polyline": []
        }

    # 座標點清單：index 0 為 depot，其餘為站點
    coords = [[float(payload.depot.lon), float(payload.depot.lat)]]
    for s in valid_stops:
        coords.append([float(s.lon), float(s.lat)])

    dist_matrix = fetch_osrm_table(coords)
    node_indices = list(range(len(coords)))
    ordered_indices = open_tsp_2opt(node_indices, dist_matrix, start_idx=0)

    # 組裝依序的站點及幾何路徑
    ordered_coords = [[payload.depot.lon, payload.depot.lat]]
    sequence_result = []

    for idx in ordered_indices:
        if idx == 0:
            sequence_result.append({
                "type": "DEPOT",
                "id": payload.depot.id,
                "name": payload.depot.name,
                "lat": payload.depot.lat,
                "lon": payload.depot.lon
            })
        else:
            stop = valid_stops[idx - 1]
            sequence_result.append({
                "type": "STOP",
                "stop_id": stop.stop_id,
                "order_id": stop.order_id,
                "name": stop.name,
                "address": stop.address,
                "lat": stop.lat,
                "lon": stop.lon
            })
            ordered_coords.append([stop.lon, stop.lat])

    route_geo = get_osrm_geometry(ordered_coords)

    return {
        "status": "success",
        "order_id": payload.order_id,
        "total_stops": len(valid_stops),
        "total_distance_km": route_geo["distance_km"],
        "estimated_duration_min": route_geo["duration_min"],
        "sequence": sequence_result,
        "polyline": route_geo["geometry"]
    }

if __name__ == "__main__":
    import uvicorn
    uvicorn.run(app, host="127.0.0.1", port=8000)