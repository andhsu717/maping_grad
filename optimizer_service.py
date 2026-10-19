"""
智慧物流路徑最佳化服務 (Multi-Depot TSP + OSRM Step-by-Step 道路分類車速計算)
更新重點：
1. 站點聚合合併：自動將距離過近 (<= 30m) 或同地址站點合併為單一停靠節點
2. 道路類型車速演算法：解析 OSRM steps 所經街道，主要幹道(大道/路)以 60 km/h 為主做行車時間計算
3. 純路徑行駛時間計算：排除站點作業時間，專注於行車預估時間
4. 箭頭顏色狀態：預設全為 GREEN (綠色暢通)，預留 ORANGE / RED 介面
"""
import math
import bisect
import os
import requests
from concurrent.futures import ThreadPoolExecutor
from typing import List, Dict, Any, Optional, Tuple
from fastapi import FastAPI, HTTPException
from fastapi.middleware.cors import CORSMiddleware
from pydantic import BaseModel

import tdx_client

app = FastAPI(title="Logistics Route Optimizer API")

app.add_middleware(
    CORSMiddleware,
    allow_origins=["*"],
    allow_credentials=True,
    allow_methods=["*"],
    allow_headers=["*"],
)

OSRM_BASE_URL = os.environ.get("OSRM_BASE_URL", "http://127.0.0.1:5000")
HEADERS = {
    "User-Agent": "SmartLogisticsApp/2.0 (Python-FastAPI-Client)",
    "Accept": "application/json"
}

# ================= 道路分類與車速規則 =================
ROAD_RULES = {
    "HIGHWAY": {
        "name": "國道/快速公路/高架道路",
        "speed_cap_kmh": 90.0,
        "keywords": ["國道", "快速", "高架", "台6", "台7", "台8", "highway", "expressway", "freeway"]
    },
    "ARTERIAL": {
        "name": "主要幹道/大道",
        "speed_cap_kmh": 60.0,  # 依需求：大道路以 60 km/h 為主
        "keywords": ["大道", "路", "外環", "特一號", "avenue", "boulevard", "road"]
    },
    "STREET": {
        "name": "一般市街",
        "speed_cap_kmh": 35.0,
        "keywords": ["街", "street"]
    },
    "ALLEY": {
        "name": "巷弄細部路網",
        "speed_cap_kmh": 20.0,
        "keywords": ["巷", "弄", "小路", "lane", "alley"]
    }
}

def classify_street(name: str, ref: str = "") -> str:
    text = f"{name} {ref}".strip().lower()
    # 優先級 1：巷弄 (巷弄常附於路名後，如「中山路一段50巷」，優先判定為巷弄)
    if any(k in text for k in ROAD_RULES["ALLEY"]["keywords"]):
        return "ALLEY"
    # 優先級 2：國道、快速道路、高架
    if any(k in text for k in ROAD_RULES["HIGHWAY"]["keywords"]):
        return "HIGHWAY"
    # 優先級 3：街
    if any(k in text for k in ROAD_RULES["STREET"]["keywords"]):
        return "STREET"
    # 優先級 4：大道路或預設幹道 (60 km/h)
    return "ARTERIAL"

# ================= TDX 車輛偵測器 (VD) 覆蓋設定 =================
EXTEND_M = 500.0                 # 沿路線往兩側延伸的上限 (公尺)
COVERAGE_THRESHOLD_M = 50.0      # VD 離路線多近視為「在路徑上」(公尺)
# 視為「轉彎 (含直角彎換路)」的 maneuver 類型
TURN_MANEUVER_TYPES = {
    "turn", "roundabout", "rotary", "merge", "fork",
    "end of road", "on ramp", "off ramp", "ramp",
}
# 路名改變且伴隨明顯轉向時，亦視為轉彎
TURN_NEW_NAME_MODIFIERS = {"left", "right", "sharp left", "sharp right", "uturn"}

# 顏色（與前端左欄色票一致）
VD_COLOR_HEX = {"GREEN": "#16a34a", "ORANGE": "#ea580c", "RED": "#dc2626", "GRAY": "#94a3b8"}


def is_turn_step(step: Dict[str, Any]) -> bool:
    """判斷 OSRM step 的進入動作是否屬於「轉彎」。"""
    maneuver = step.get("maneuver") or {}
    mtype = str(maneuver.get("type", "")).lower()
    modifier = str(maneuver.get("modifier", "")).lower()
    if mtype in TURN_MANEUVER_TYPES:
        return True
    if mtype == "new name" and modifier in TURN_NEW_NAME_MODIFIERS:
        return True
    return False


def classify_vd_color(speed: Optional[float]) -> str:
    """（保留相容）僅以速度判斷綠/灰；實際顏色由 tdx_client 依官方門檻提供。"""
    if speed is not None and speed > 0:
        return "GREEN"
    return "GRAY"


def _road_name_at(segments: List[List[Any]], s: float) -> str:
    """取沿線里程 s 所在（或最近）路段的街名；無資料回空字串。"""
    if not segments:
        return ""
    for a, b, name in segments:
        if a <= s <= b:
            return str(name)
    # 投影點落在區間縫隙（如幾何斷點）時取最近者
    best_name = ""
    best_dist = float("inf")
    for a, b, name in segments:
        d = a - s if s < a else (s - b if s > b else 0.0)
        if d < best_dist:
            best_dist = d
            best_name = str(name)
    return best_name if best_dist <= 30.0 else ""


def _road_name_compatible(vd_name: str, seg_name: str) -> bool:
    """VD 路名與路線路段名是否相容（防陸橋吃到下方道路偵測器）。

    - 任一為空 → 相容（退回純距離比對，不誤殺）
    - 去空白後完全相等，或一方為另一方前綴 → 相容
      （如『中山路』↔『中山路二段』、『市民大道』↔『市民大道三段』）
    - 其餘（『中山路二段』↔『中山路三段』、『中正路』↔『中山路』）→ 不相容
    """
    a = str(vd_name or "").strip().replace(" ", "")
    b = str(seg_name or "").strip().replace(" ", "")
    if not a or not b:
        return True
    return a == b or a.startswith(b) or b.startswith(a)


def _project_point_to_path(pt_lat: float, pt_lon: float,
                           path: List[List[float]], cum: List[float]) -> Tuple[float, float]:
    """將點投影到折線上，回傳 (最近水平距離 m, 沿線累積里程 m)。"""
    lat0 = math.radians(pt_lat)
    mx = 111320.0 * math.cos(lat0)
    my = 110540.0
    px = pt_lon * mx
    py = pt_lat * my
    best_off = float("inf")
    best_s = 0.0
    for i in range(len(path) - 1):
        ax = path[i][1] * mx; ay = path[i][0] * my
        bx = path[i + 1][1] * mx; by = path[i + 1][0] * my
        dx = bx - ax; dy = by - ay
        seg2 = dx * dx + dy * dy
        t = 0.0 if seg2 <= 0 else ((px - ax) * dx + (py - ay) * dy) / seg2
        t = max(0.0, min(1.0, t))
        cx = ax + dx * t; cy = ay + dy * t
        off = math.hypot(px - cx, py - cy)
        if off < best_off:
            best_off = off
            best_s = cum[i] + (cum[i + 1] - cum[i]) * t
    return best_off, best_s


def _path_point_at(path: List[List[float]], cum: List[float], d: float) -> List[float]:
    total = cum[-1]
    if d <= 0:
        return [path[0][0], path[0][1]]
    if d >= total:
        return [path[-1][0], path[-1][1]]
    i = bisect.bisect_right(cum, d) - 1
    i = max(0, min(i, len(path) - 2))
    seg = cum[i + 1] - cum[i]
    t = 0.0 if seg <= 0 else (d - cum[i]) / seg
    lat = path[i][0] + (path[i + 1][0] - path[i][0]) * t
    lng = path[i][1] + (path[i + 1][1] - path[i][1]) * t
    return [lat, lng]


def _slice_path(path: List[List[float]], cum: List[float], d0: float, d1: float) -> List[List[float]]:
    """取出折線由里程 d0 到 d1 的子折線。"""
    if not path or len(path) < 2 or not cum:
        return []
    d0 = max(0.0, d0)
    d1 = min(cum[-1], d1)
    if d1 <= d0:
        return []
    out = [_path_point_at(path, cum, d0)]
    for i in range(len(path)):
        if d0 < cum[i] < d1:
            out.append([path[i][0], path[i][1]])
    out.append(_path_point_at(path, cum, d1))
    return out


def _bearing_deg(lat1: float, lon1: float, lat2: float, lon2: float) -> float:
    """由點1指向點2的初始方位角（度，0=正北，順時針）。"""
    phi1 = math.radians(lat1)
    phi2 = math.radians(lat2)
    dlon = math.radians(lon2 - lon1)
    x = math.sin(dlon) * math.cos(phi2)
    y = math.cos(phi1) * math.sin(phi2) - math.sin(phi1) * math.cos(phi2) * math.cos(dlon)
    return (math.degrees(math.atan2(x, y)) + 360.0) % 360.0


def _offset_latlon(lat: float, lon: float, bearing_deg: float, dist_m: float) -> List[float]:
    """由 (lat,lon) 沿方位角前進 dist_m，回傳新點 [lat, lon]。"""
    R = 6371000.0
    ang = dist_m / R
    brg = math.radians(bearing_deg)
    phi1 = math.radians(lat)
    lam1 = math.radians(lon)
    phi2 = math.asin(math.sin(phi1) * math.cos(ang)
                     + math.cos(phi1) * math.sin(ang) * math.cos(brg))
    lam2 = lam1 + math.atan2(math.sin(brg) * math.sin(ang) * math.cos(phi1),
                             math.cos(ang) - math.sin(phi1) * math.sin(phi2))
    return [math.degrees(phi2), (math.degrees(lam2) + 540.0) % 360.0 - 180.0]


def compute_vd_overlay(profile: Dict[str, Any], cities: List[str],
                       force_live: bool = False) -> Tuple[List[Dict[str, Any]], List[Dict[str, Any]]]:
    """比對 VD 與路線，產生標記點與「±500m、遇轉彎截斷」的覆蓋子折線。"""
    path = profile.get("path") or []
    cum = profile.get("cum") or []
    turns = profile.get("turn_boundaries") or []
    leg_bounds = profile.get("leg_boundaries") or []
    road_segs = profile.get("road_segments") or []
    points: List[Dict[str, Any]] = []
    coverage: List[Dict[str, Any]] = []
    if len(path) < 2 or not cities:
        return points, coverage

    live = tdx_client.live_map(cities, force_live=force_live)
    seen: set = set()
    for city in cities:
        for vd in tdx_client.get_city_vds(city):
            vdid = str(vd.get("VDID", ""))
            lat = vd.get("PositionLat")
            lon = vd.get("PositionLon")
            if not vdid or lat is None or lon is None or vdid in seen:
                continue
            seen.add(vdid)

            lat = float(lat); lon = float(lon)
            off, s = _project_point_to_path(lat, lon, path, cum)

            entry = live.get(vdid)
            if entry is not None and entry.get("status") != 0:
                continue
            speed = entry.get("speed") if entry else None
            color = entry.get("color") if entry else "GRAY"
            level = entry.get("level") if entry else None
            level_label = entry.get("level_name") if entry else ""
            levels = entry.get("levels") if entry else None
            on_route = off <= COVERAGE_THRESHOLD_M
            # 路名比對：距離合格後再比街名，防止陸橋路線吃到下方道路的 VD
            if on_route and road_segs:
                on_route = _road_name_compatible(vd.get("RoadName", ""), _road_name_at(road_segs, s))

            points.append({
                "vdid": vdid,
                "lat": lat,
                "lon": lon,
                "road": vd.get("RoadName", ""),
                "road_class": vd.get("RoadClass"),
                "speed": speed,
                "speed_valid": entry.get("speed_valid") if entry else False,
                "color": color,
                "level": level,
                "level_name": level_label,
                "levels": levels,
                "status": entry.get("status") if entry else None,
                "collect_time": entry.get("collect_time") if entry else None,
                "on_route": on_route,
            })

            if not on_route:
                continue

            back_turn = max([t for t in turns if t <= s], default=0.0)
            fwd_turn = min([t for t in turns if t >= s], default=cum[-1])
            back_len = min(EXTEND_M, s - back_turn)
            fwd_len = min(EXTEND_M, fwd_turn - s)
            seg = _slice_path(path, cum, s - back_len, s + fwd_len)
            if len(seg) < 2:
                continue

            leg_index = max(0, len(leg_bounds) - 1)
            for i, b in enumerate(leg_bounds):
                if s <= b:
                    leg_index = i
                    break

            coverage.append({
                "vdid": vdid,
                "leg_index": leg_index,
                "s_m": round(s, 1),
                "path": seg,
                "back_m": round(back_len, 1),
                "fwd_m": round(fwd_len, 1),
                "color": color,
                "level": level,
                "levels": levels,
                "speed": speed,
                "collect_time": entry.get("collect_time") if entry else None,
            })
    return points, coverage

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
    date: Optional[str] = ""
    order_id: str
    depot: DepotItem
    stops: List[DeliveryStop]

# ================= 經緯度距離與站點合併 =================
def haversine_distance_meters(lat1: float, lon1: float, lat2: float, lon2: float) -> float:
    R = 6371000  # 公尺
    dlat = math.radians(lat2 - lat1)
    dlon = math.radians(lon2 - lon1)
    a = math.sin(dlat / 2)**2 + math.cos(math.radians(lat1)) * math.cos(math.radians(lat2)) * math.sin(dlon / 2)**2
    c = 2 * math.atan2(math.sqrt(a), math.sqrt(1 - a))
    return R * c

def merge_stops_by_proximity_or_address(stops: List[DeliveryStop], threshold_meters: float = 30.0) -> List[Dict[str, Any]]:
    clusters: List[Dict[str, Any]] = []

    for stop in stops:
        merged = False
        norm_addr = (stop.address or "").strip().replace(" ", "").replace("-", "")

        for cluster in clusters:
            clust_addr = (cluster["address"] or "").strip().replace(" ", "").replace("-", "")
            dist = haversine_distance_meters(stop.lat, stop.lon, cluster["lat"], cluster["lon"])

            # 判定條件：同一地址 (且非空) 或距離小於等於閥值
            if (norm_addr and clust_addr and norm_addr == clust_addr) or (dist <= threshold_meters):
                cluster["orders"].append({
                    "stop_id": stop.stop_id,
                    "order_id": stop.order_id,
                    "name": stop.name,
                    "address": stop.address,
                    "item_name": stop.item_name,
                    "weight_kg": stop.weight_kg or 0.0,
                    "volume_cbm": stop.volume_cbm or 0.0
                })
                cluster["total_weight_kg"] += (stop.weight_kg or 0.0)
                cluster["total_volume_cbm"] += (stop.volume_cbm or 0.0)
                merged = True
                break

        if not merged:
            clusters.append({
                "cluster_id": f"CLUST_{len(clusters) + 1}",
                "lat": stop.lat,
                "lon": stop.lon,
                "address": stop.address or "",
                "primary_name": stop.name or f"站點_{len(clusters) + 1}",
                "primary_stop_id": stop.stop_id,
                "total_weight_kg": stop.weight_kg or 0.0,
                "total_volume_cbm": stop.volume_cbm or 0.0,
                "orders": [{
                    "stop_id": stop.stop_id,
                    "order_id": stop.order_id,
                    "name": stop.name,
                    "address": stop.address,
                    "item_name": stop.item_name,
                    "weight_kg": stop.weight_kg or 0.0,
                    "volume_cbm": stop.volume_cbm or 0.0
                }]
            })

    return clusters

# ================= 距離矩陣與 TSP 求解 =================
def compute_haversine_matrix(coords: List[List[float]]) -> List[List[float]]:
    n = len(coords)
    matrix = [[0.0] * n for _ in range(n)]
    for i in range(n):
        for j in range(i + 1, n):
            dist = haversine_distance_meters(coords[i][1], coords[i][0], coords[j][1], coords[j][0])
            matrix[i][j] = dist
            matrix[j][i] = dist
    return matrix

def fetch_osrm_table(coords: List[List[float]]) -> List[List[float]]:
    if len(coords) > 100:
        return compute_haversine_matrix(coords)
    coords_str = ";".join([f"{float(c[0]):.6f},{float(c[1]):.6f}" for c in coords])
    url = f"{OSRM_BASE_URL}/table/v1/driving/{coords_str}?annotations=distance"
    try:
        resp = requests.get(url, headers=HEADERS, timeout=5)
        if resp.status_code == 200:
            data = resp.json()
            if data.get("code") == "Ok" and "distances" in data:
                return data["distances"]
    except requests.exceptions.RequestException:
        pass
    return compute_haversine_matrix(coords)

def open_tsp_2opt(node_indices: List[int], dist_matrix: List[List[float]], start_idx: int = 0) -> List[int]:
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

# ================= 道路類型車速解析與路徑幾何計算 =================
def compute_route_with_road_speed(ordered_coords: List[List[float]]) -> Dict[str, Any]:
    if len(ordered_coords) < 2:
        return {
            "geometry": [],
            "total_distance_km": 0,
            "total_duration_min": 0,
            "legs": [],
            "profile": {"path": [], "cum": [], "turn_boundaries": [], "leg_boundaries": [], "road_segments": []}
        }

    # OSRM Route 支援最多約 80~100 點，開啟 steps=true 以獲取街道名稱與各步細節
    coords_str = ";".join([f"{float(c[0]):.6f},{float(c[1]):.6f}" for c in ordered_coords])
    url = f"{OSRM_BASE_URL}/route/v1/driving/{coords_str}?overview=full&geometries=geojson&steps=true"
    
    try:
        resp = requests.get(url, headers=HEADERS, timeout=5)
        if resp.status_code == 200:
            data = resp.json()
            if data.get("routes"):
                route_info = data["routes"][0]
                latlng_path = [[pt[1], pt[0]] for pt in route_info["geometry"]["coordinates"]]

                legs_result = []
                total_adjusted_duration_s = 0.0
                total_distance_m = 0.0

                # 路線剖面：沿線座標 + 累積里程 + 轉彎/分段邊界（供 VD 覆蓋段計算）
                prof_path: List[List[float]] = []
                prof_cum: List[float] = [0.0]
                turn_boundaries: List[float] = []
                leg_boundaries: List[float] = []
                road_segments: List[List[Any]] = []  # [起m, 終m, 街名]（供 VD 路名比對）

                def _append_prof(lat: float, lng: float):
                    if prof_path:
                        prev = prof_path[-1]
                        if abs(prev[0] - lat) < 1e-9 and abs(prev[1] - lng) < 1e-9:
                            return
                        prof_cum.append(prof_cum[-1] + haversine_distance_meters(prev[0], prev[1], lat, lng))
                    prof_path.append([lat, lng])

                for leg_idx, leg in enumerate(route_info.get("legs", [])):
                    leg_dist_m = 0.0
                    leg_dur_s = 0.0
                    street_names = []

                    for step in leg.get("steps", []):
                        s_dist = step.get("distance", 0.0)
                        s_dur = step.get("duration", 0.0)
                        st_name = step.get("name", "")
                        st_ref = step.get("ref", "")

                        if st_name and st_name not in street_names:
                            street_names.append(st_name)

                        seg_start_m = prof_cum[-1]

                        # 進入此 step 若為轉彎，記錄轉彎邊界（累積里程）
                        if prof_path and is_turn_step(step):
                            turn_boundaries.append(prof_cum[-1])

                        # 累積此 step 的幾何點
                        for pt in (step.get("geometry") or {}).get("coordinates", []):
                            _append_prof(pt[1], pt[0])

                        # 街名區間（供 VD 路名比對；ref 可補名稱不足，如台61線）
                        seg_name = st_name or st_ref
                        if seg_name and prof_cum[-1] > seg_start_m:
                            road_segments.append([seg_start_m, prof_cum[-1], seg_name])

                        # 判定道路類型並取得速限上限 (大道路為 60 km/h)
                        rtype = classify_street(st_name, st_ref)
                        cap_speed_kmh = ROAD_RULES[rtype]["speed_cap_kmh"]
                        cap_speed_mps = cap_speed_kmh / 3.6

                        # 原始車速 (m/s)
                        orig_speed_mps = (s_dist / s_dur) if s_dur > 0 else (cap_speed_mps * 0.8)
                        effective_speed_mps = min(orig_speed_mps, cap_speed_mps)
                        adj_step_dur = (s_dist / effective_speed_mps) if effective_speed_mps > 0 else s_dur

                        leg_dist_m += s_dist
                        leg_dur_s += adj_step_dur

                    total_distance_m += leg_dist_m
                    total_adjusted_duration_s += leg_dur_s
                    leg_boundaries.append(prof_cum[-1] if len(prof_path) >= 2 else total_distance_m)

                    # 箭頭顏色預設綠色 (GREEN)，之後可依 TDX 即時車流更新
                    legs_result.append({
                        "leg_index": leg_idx,
                        "from_seq": leg_idx,
                        "to_seq": leg_idx + 1,
                        "distance_km": round(leg_dist_m / 1000.0, 2),
                        "duration_min": round(leg_dur_s / 60.0, 1),
                        "main_street": " → ".join(street_names[:3]) if street_names else "一般街道",
                        "traffic_status": "GREEN"
                    })

                # 若 step 幾何缺失，改用路線整體幾何建立剖面
                if len(prof_path) < 2:
                    fb_path = latlng_path if len(latlng_path) >= 2 else [[p[1], p[0]] for p in ordered_coords]
                    fb_cum = [0.0]
                    for i in range(1, len(fb_path)):
                        a, b = fb_path[i - 1], fb_path[i]
                        fb_cum.append(fb_cum[-1] + haversine_distance_meters(a[0], a[1], b[0], b[1]))
                    prof_path, prof_cum = fb_path, fb_cum
                    turn_boundaries = []
                    leg_boundaries = [prof_cum[-1]]

                return {
                    "geometry": latlng_path,
                    "total_distance_km": round(total_distance_m / 1000.0, 2),
                    "total_duration_min": round(total_adjusted_duration_s / 60.0, 1),
                    "legs": legs_result,
                    "profile": {
                        "path": prof_path,
                        "cum": prof_cum,
                        "turn_boundaries": turn_boundaries,
                        "leg_boundaries": leg_boundaries,
                        "road_segments": road_segments,
                    },
                }
    except Exception as e:
        print(f"OSRM Route request error: {e}")

    # Fallback 直線距離運算 (以大道路 60 km/h 估算)
    fallback_path = [[c[1], c[0]] for c in ordered_coords]
    legs_result = []
    total_dist_m = 0.0
    total_dur_s = 0.0

    for i in range(len(ordered_coords) - 1):
        c1, c2 = ordered_coords[i], ordered_coords[i+1]
        dist_m = haversine_distance_meters(c1[1], c1[0], c2[1], c2[0]) * 1.3  # 道路彎曲折減率
        dur_s = (dist_m / (60.0 / 3.6))  # 以 60 km/h 計算
        total_dist_m += dist_m
        total_dur_s += dur_s
        legs_result.append({
            "leg_index": i,
            "from_seq": i,
            "to_seq": i + 1,
            "distance_km": round(dist_m / 1000.0, 2),
            "duration_min": round(dur_s / 60.0, 1),
            "main_street": "市區主要道路",
            "traffic_status": "GREEN"
        })

    fb_cum = [0.0]
    for i in range(1, len(fallback_path)):
        a, b = fallback_path[i - 1], fallback_path[i]
        fb_cum.append(fb_cum[-1] + haversine_distance_meters(a[0], a[1], b[0], b[1]))

    return {
        "geometry": fallback_path,
        "total_distance_km": round(total_dist_m / 1000.0, 2),
        "total_duration_min": round(total_dur_s / 60.0, 1),
        "legs": legs_result,
        "profile": {
            "path": fallback_path,
            "cum": fb_cum,
            "turn_boundaries": [],
            "leg_boundaries": [fb_cum[-1]] if fb_cum else [],
            "road_segments": [],
        },
    }

# ================= 單行程最佳化端點 =================
@app.post("/api/optimize-single-trip")
def optimize_single_trip(payload: SingleTripRequest, force: bool = False):
    valid_stops = [s for s in payload.stops if s.lat and s.lon]
    if not valid_stops:
        return {
            "status": "success",
            "order_id": payload.order_id,
            "total_stops": 0,
            "total_distance_km": 0,
            "estimated_duration_min": 0,
            "sequence": [],
            "legs": [],
            "polyline": [],
            "counties": [],
            "vd_points": [],
            "vd_coverage": [],
            "route_meta": {"cum": [], "turn_boundaries": [], "leg_boundaries": []}
        }

    # 1. 站點聚合合併 (地址相同或 <= 30m 歸於同一個點)
    clusters = merge_stops_by_proximity_or_address(valid_stops, threshold_meters=30.0)

    # 2. 座標點清單：index 0 為出發站 (depot)，其餘為合併後的停靠節點
    coords = [[float(payload.depot.lon), float(payload.depot.lat)]]
    for c in clusters:
        coords.append([float(c["lon"]), float(c["lat"])])

    # 3. 取得距離矩陣並進行 Open-TSP 求解 (起點固定為 depot)
    dist_matrix = fetch_osrm_table(coords)
    node_indices = list(range(len(coords)))
    ordered_indices = open_tsp_2opt(node_indices, dist_matrix, start_idx=0)

    # 4. 組裝依序的站點及座標清單
    ordered_coords = [[payload.depot.lon, payload.depot.lat]]
    sequence_result = []

    seq_counter = 1
    for idx in ordered_indices:
        if idx == 0:
            sequence_result.append({
                "type": "DEPOT",
                "seq_no": 0,
                "id": payload.depot.id,
                "name": payload.depot.name,
                "address": payload.depot.name,
                "lat": payload.depot.lat,
                "lon": payload.depot.lon,
                "is_merged": False,
                "merged_count": 1,
                "orders": []
            })
        else:
            cluster = clusters[idx - 1]
            sequence_result.append({
                "type": "STOP",
                "seq_no": seq_counter,
                "cluster_id": cluster["cluster_id"],
                "stop_id": cluster["primary_stop_id"],
                "name": cluster["primary_name"],
                "address": cluster["address"],
                "lat": cluster["lat"],
                "lon": cluster["lon"],
                "is_merged": len(cluster["orders"]) > 1,
                "merged_count": len(cluster["orders"]),
                "total_weight_kg": round(cluster["total_weight_kg"], 1),
                "total_volume_cbm": round(cluster["total_volume_cbm"], 2),
                "orders": cluster["orders"]
            })
            ordered_coords.append([cluster["lon"], cluster["lat"]])
            seq_counter += 1

    # 5. OSRM steps 解析道路類型車速與各 Leg 行車時間
    route_calc = compute_route_with_road_speed(ordered_coords)

    # 6. TDX 車輛偵測器：由地址判定縣市 → 取 VD 標記點與覆蓋段
    counties = tdx_client.detect_counties([s.address for s in payload.stops])
    try:
        vd_points, vd_coverage = compute_vd_overlay(route_calc.get("profile", {}), counties, force_live=force)
    except Exception as exc:  # TDX 失敗不影響路線計算
        print(f"[TDX] VD overlay 失敗: {exc}")
        vd_points, vd_coverage = [], []

    return {
        "status": "success",
        "order_id": payload.order_id,
        "date": payload.date,
        "total_stops": len(valid_stops),
        "total_clusters": len(clusters),
        "total_distance_km": route_calc["total_distance_km"],
        "estimated_duration_min": route_calc["total_duration_min"],
        "sequence": sequence_result,
        "legs": route_calc["legs"],
        "polyline": route_calc["geometry"],
        "counties": counties,
        "vd_points": vd_points,
        "vd_coverage": vd_coverage,
        "route_meta": {
            "cum": route_calc["profile"]["cum"],
            "turn_boundaries": route_calc["profile"]["turn_boundaries"],
            "leg_boundaries": route_calc["profile"]["leg_boundaries"]
        }
    }


@app.get("/api/traffic/live")
def traffic_live(counties: str = "", force: bool = False):
    """輪詢用：回傳指定縣市的 VD 即時速度與官方壅塞等級（前端據此更新顏色）。

    force=True 時跳過即時資料快取，每次皆向 TDX 重新取號誌。
    """
    codes = [c.strip() for c in counties.split(",") if c.strip()]
    data = tdx_client.live_map(codes, force_live=force)
    return {"status": "success", "counties": codes, "vds": data}


# ================= 改道備選路徑 (Phase 3) =================
REROUTE_MIN_WINDOW_M = 100.0   # P1→P2 區間過短則不提改道
REROUTE_ALT_MAX = 3            # 每條 leg 最多候選路徑數（自建 OSRM 預設上限 3，超過會回 400）
REROUTE_MIN_SAVED_MIN = 0.0    # 無省時的候選過濾並回報 NO_TIME_SAVED
REROUTE_MIN_DEVIATION_M = 5.0  # 備選與主線偏離小於此值視為假備選丟棄
REROUTE_OSRM_WORKERS = 4       # 平行查詢 OSRM 的執行緒數


def _vd_intervals_on_path(path: List[List[float]], cum: List[float],
                          vds: List[Dict[str, Any]]) -> List[List[float]]:
    """將 VD 投影到路徑，回傳合併後的覆蓋區間 [[a_m, b_m, speed_kmh], ...]。"""
    intervals: List[List[float]] = []
    total = cum[-1]
    for vd in vds:
        spd = vd.get("speed")
        if spd is None or float(spd) <= 0:
            continue
        off, s = _project_point_to_path(float(vd["lat"]), float(vd["lon"]), path, cum)
        if off > COVERAGE_THRESHOLD_M:
            continue
        a = max(0.0, s - EXTEND_M)
        b = min(total, s + EXTEND_M)
        if b > a:
            intervals.append([a, b, min(float(spd), 90.0)])
    intervals.sort(key=lambda x: x[0])
    merged: List[List[float]] = []
    for a, b, v in intervals:
        if merged and a <= merged[-1][1]:
            merged[-1][1] = max(merged[-1][1], b)
            merged[-1][2] = min(merged[-1][2], v)
        else:
            merged.append([a, b, v])
    return merged


def _duration_over(merged: List[List[float]], d0: float, d1: float,
                   nom_speed_mps: float) -> float:
    """里程 [d0,d1] 的行駛時間（秒）：基準速度 + VD 區間以實際速度替代。"""
    if d1 <= d0 or nom_speed_mps <= 0:
        return 0.0
    total = 0.0
    cursor = d0
    for a, b, v in merged:
        if b <= d0 or a >= d1:
            continue
        aa = max(a, d0)
        bb = min(b, d1)
        if aa > cursor:
            total += (aa - cursor) / nom_speed_mps
        vs = max(float(v) / 3.6, 0.5)
        total += (bb - aa) / vs
        cursor = bb
    if cursor < d1:
        total += (d1 - cursor) / nom_speed_mps
    return total


def _max_deviation_m(alt_path: List[List[float]], main_path: List[List[float]],
                     main_cum: List[float]) -> Tuple[float, List[float], List[float]]:
    """候選路徑相對主線的最大偏離距離，以及 (第一個偏離點, 最後一個偏離點)。"""
    max_off = 0.0
    exit_at: List[float] = []
    rejoin_at: List[float] = []
    step = max(1, len(alt_path) // 60)
    for i in range(0, len(alt_path), step):
        pt = alt_path[i]
        off, _ = _project_point_to_path(pt[0], pt[1], main_path, main_cum)
        if off > max_off:
            max_off = off
        if off > COVERAGE_THRESHOLD_M:
            if not exit_at:
                exit_at = pt
            rejoin_at = pt
    if not exit_at:
        exit_at = alt_path[0]
        rejoin_at = alt_path[-1]
    return max_off, exit_at, rejoin_at


def _fetch_osrm_alternatives(pt1: List[float], pt2: List[float]) -> List[Dict[str, Any]]:
    """向 OSRM 查 P1→P2 備選路徑；失敗或無結果回 []。"""
    try:
        url = (f"{OSRM_BASE_URL}/route/v1/driving/"
               f"{pt1[1]:.6f},{pt1[0]:.6f};{pt2[1]:.6f},{pt2[0]:.6f}"
               f"?alternatives={REROUTE_ALT_MAX}&overview=full&geometries=geojson&steps=false")
        resp = requests.get(url, headers=HEADERS, timeout=8)
        if resp.status_code == 200:
            return resp.json().get("routes", []) or []
        print(f"[reroute] OSRM {resp.status_code}: {resp.text[:200]}")
    except Exception:
        pass
    return []


class RouteAltVd(BaseModel):
    vdid: str = ""
    lat: float
    lon: float
    speed: Optional[float] = None


class RouteAltLeg(BaseModel):
    leg_index: int
    distance_km: float = 0.0
    duration_min: float = 0.0
    base_duration_min: Optional[float] = None


class RouteAlternativesRequest(BaseModel):
    polyline: List[List[float]]
    cum: List[float] = []
    turn_boundaries: List[float] = []
    leg_boundaries: List[float] = []
    coverage: List[Dict[str, Any]] = []
    vds: List[RouteAltVd] = []
    counties: List[str] = []
    legs: List[RouteAltLeg] = []


@app.post("/api/route/alternatives")
def route_alternatives(req: RouteAlternativesRequest):
    """對所有壅塞 leg 推算改道點 (P1) 與回歸點 (P2)，回全量候選與無法改道原因。

    skip_reason: null=有候選；"WINDOW_TOO_SHORT"=路徑過短；
                 "NO_ALTERNATIVE"=無替代路徑（OSRM 失敗/查無路徑/皆與原路重疊）；
                 "NO_TIME_SAVED"=無法節省時間。
    """
    path = req.polyline or []
    if len(path) < 2:
        return {"status": "success", "legs": []}
    cum = req.cum or []
    if len(cum) != len(path):
        cum = [0.0]
        for i in range(1, len(path)):
            cum.append(cum[-1] + haversine_distance_meters(
                path[i - 1][0], path[i - 1][1], path[i][0], path[i][1]))
    total_m = cum[-1]
    if total_m <= 0:
        return {"status": "success", "legs": []}

    turns = req.turn_boundaries or []
    leg_bounds = req.leg_boundaries or []
    vds = [v.model_dump() if hasattr(v, "model_dump") else v.dict() for v in req.vds]
    legs_info = {l.leg_index: (l.model_dump() if hasattr(l, "model_dump") else l.dict())
                 for l in req.legs}

    # 1) 依有效顏色合併每條 leg 的橘/紅壅塞區間
    by_leg: Dict[int, Dict[str, Any]] = {}
    for cov in req.coverage:
        color = cov.get("color")
        if color not in ("ORANGE", "RED"):
            continue
        li = cov.get("leg_index")
        if li is None:
            continue
        try:
            s = float(cov.get("s_m", 0.0))
            a = s - float(cov.get("back_m", 0.0))
            b = s + float(cov.get("fwd_m", 0.0))
        except (TypeError, ValueError):
            continue
        li = int(li)
        slot = by_leg.setdefault(li, {"a": a, "b": b, "worst": color})
        slot["a"] = min(slot["a"], a)
        slot["b"] = max(slot["b"], b)
        if color == "RED":
            slot["worst"] = "RED"

    ranked = sorted(by_leg.items(),
                    key=lambda kv: (-(kv[1]["worst"] == "RED"), -(kv[1]["b"] - kv[1]["a"])))
    if not ranked:
        return {"status": "success", "legs": []}

    merged_main = _vd_intervals_on_path(path, cum, vds)
    main_total_s = sum(float(x.get("duration_min") or 0.0) * 60.0
                       for x in legs_info.values())

    def _leg_result(li: int, slot: Dict[str, Any]) -> Dict[str, Any]:
        a, b = slot["a"], slot["b"]
        base: Dict[str, Any] = {
            "leg_index": li,
            "a_m": round(a, 1),
            "b_m": round(b, 1),
            "current_eta_min": round(main_total_s / 60.0, 1),
        }
        # 2) P1 = 壅塞前最後一個轉彎路口；P2 = 壅塞後第一個轉彎路口（無緩衝）
        p1 = max([t for t in turns if t <= a], default=0.0)
        p2 = min([t for t in turns if t >= b], default=total_m)
        if p2 - p1 < REROUTE_MIN_WINDOW_M:
            return {**base, "alternatives": [], "skip_reason": "WINDOW_TOO_SHORT"}
        pt1 = _path_point_at(path, cum, p1)
        pt2 = _path_point_at(path, cum, p2)

        # 3) OSRM 取 P1→P2 備選
        routes = _fetch_osrm_alternatives(pt1, pt2)
        if not routes:
            return {**base, "alternatives": [], "skip_reason": "NO_ALTERNATIVE"}

        # 主線 leg 的名目速度與當前（含壅塞）總時間
        leg_info = legs_info.get(li, {})
        leg_start = leg_bounds[li - 1] if 0 < li < len(leg_bounds) else 0.0
        leg_end = leg_bounds[li] if li < len(leg_bounds) else total_m
        leg_base_min = float(leg_info.get("base_duration_min")
                             or leg_info.get("duration_min") or 0.0)
        leg_nom = ((leg_end - leg_start) / (leg_base_min * 60.0)) if leg_base_min > 0 else 15.0
        main_p1p2_s = _duration_over(merged_main, p1, p2, leg_nom)
        base_p1p2_s = (p2 - p1) / leg_nom if leg_nom > 0 else 0.0
        total_s = main_total_s if main_total_s > 0 else total_m / max(leg_nom, 1.0)

        alts: List[Dict[str, Any]] = []
        time_filtered = False
        for rt in routes[:REROUTE_ALT_MAX]:
            alt_path = [[c[1], c[0]] for c in (rt.get("geometry") or {}).get("coordinates", [])]
            if len(alt_path) < 2:
                continue
            alt_cum = [0.0]
            for i in range(1, len(alt_path)):
                alt_cum.append(alt_cum[-1] + haversine_distance_meters(
                    alt_path[i - 1][0], alt_path[i - 1][1], alt_path[i][0], alt_path[i][1]))
            alt_total_m = alt_cum[-1]
            if alt_total_m <= 0:
                continue
            # 幾何過濾：與主線幾乎重疊（假備選）才丟棄
            max_off, exit_at, rejoin_at = _max_deviation_m(alt_path, path, cum)
            if max_off <= REROUTE_MIN_DEVIATION_M:
                continue
            alt_osrm_s = float(rt.get("duration") or 0.0)
            alt_nom = (alt_total_m / alt_osrm_s) if alt_osrm_s > 0 else 15.0
            merged_alt = _vd_intervals_on_path(alt_path, alt_cum, vds)
            alt_p1p2_s = _duration_over(merged_alt, 0.0, alt_total_m, alt_nom)
            cand_total_s = total_s - main_p1p2_s + alt_p1p2_s
            saved_min = (total_s - cand_total_s) / 60.0
            if saved_min <= REROUTE_MIN_SAVED_MIN:
                time_filtered = True
                continue
            alts.append({
                "seg_polyline": alt_path,
                "p1_m": round(p1, 1),
                "p2_m": round(p2, 1),
                "leg_start_m": round(leg_start, 1),
                "leg_end_m": round(leg_end, 1),
                "main_p1p2_min": round(main_p1p2_s / 60.0, 1),
                "base_p1p2_min": round(base_p1p2_s / 60.0, 1),
                "alt_p1p2_min": round(alt_p1p2_s / 60.0, 1),
                "eta_min": round(cand_total_s / 60.0, 1),
                "saved_min": round(saved_min, 1),
                "distance_km": round(alt_total_m / 1000.0, 2),
                "exit_at": exit_at,
                "rejoin_at": rejoin_at
            })

        if not alts:
            reason = "NO_TIME_SAVED" if time_filtered else "NO_ALTERNATIVE"
            return {**base, "alternatives": [], "skip_reason": reason}
        return {**base, "alternatives": alts, "skip_reason": None}

    workers = min(REROUTE_OSRM_WORKERS, len(ranked))
    with ThreadPoolExecutor(max_workers=workers) as ex:
        out_legs = list(ex.map(lambda kv: _leg_result(kv[0], kv[1]), ranked))

    return {"status": "success", "legs": out_legs}


# 首頁回傳（必須在所有 API 路由之後註冊）：
# ngrok 公開時只需一條隧道指向 8000，此路由讓 / 直接回傳 index.html。
# 注意：只回傳 index.html 單檔，不掛載整個目錄（避免 key.env 等敏感檔案外流）。
BASE_DIR = os.path.dirname(os.path.abspath(__file__))


@app.get("/", include_in_schema=False)
def serve_index():
    from fastapi.responses import FileResponse
    return FileResponse(os.path.join(BASE_DIR, "index.html"))


if __name__ == "__main__":
    import uvicorn
    uvicorn.run(app, host="127.0.0.1", port=8000)