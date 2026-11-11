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
                           path: List[List[float]], cum: List[float],
                           min_s: float = 0.0) -> Tuple[float, float]:
    """將點投影到折線上，回傳 (最近水平距離 m, 沿線累積里程 m)。
    min_s > 0 時只搜尋里程 ≥ min_s 的部分（依序投影站點，避免路線重疊時投影到錯的一趟）。"""
    lat0 = math.radians(pt_lat)
    mx = 111320.0 * math.cos(lat0)
    my = 110540.0
    px = pt_lon * mx
    py = pt_lat * my
    best_off = float("inf")
    best_s = 0.0
    for i in range(len(path) - 1):
        if min_s > 0 and cum[i + 1] < min_s:
            continue
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


def compute_vd_overlay(profile: Dict[str, Any], cities: List[str],
                       force_live: bool = False) -> Tuple[List[Dict[str, Any]], List[Dict[str, Any]]]:
    """比對 VD 與路線，產生標記點與「±500m、遇轉彎截斷、相鄰中點平分」的覆蓋子折線。"""
    path = profile.get("path") or []
    cum = profile.get("cum") or []
    turns = profile.get("turn_boundaries") or []
    leg_bounds = profile.get("leg_boundaries") or []
    road_segs = profile.get("road_segments") or []
    points: List[Dict[str, Any]] = []
    hits: List[Dict[str, Any]] = []
    if len(path) < 2 or not cities:
        return points, []

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

            entry = live.get(vdid) or tdx_client.empty_entry(vd)
            on_route = off <= COVERAGE_THRESHOLD_M
            # 路名比對：距離合格後再比街名，防止陸橋路線吃到下方道路的 VD
            if on_route and road_segs:
                on_route = _road_name_compatible(vd.get("RoadName", ""), _road_name_at(road_segs, s))
            # 行進方向比對：只取與路線同方向的車道；僅量測對向的偵測器不算在路線上
            dir_links: Optional[List[str]] = None
            dir_mismatch = False
            if on_route:
                dir_links = tdx_client.links_for_heading(entry.get("links") or {}, _bearing_at(path, cum, s))
                if dir_links is None:
                    on_route = False
                    dir_mismatch = True
            reading = tdx_client.direction_reading(entry, dir_links) if on_route else entry
            speed = reading.get("speed")
            color = reading.get("color") or "GRAY"
            level = reading.get("level")
            levels = reading.get("levels")

            points.append({
                "vdid": vdid,
                "lat": lat,
                "lon": lon,
                "road": vd.get("RoadName", ""),
                "road_class": vd.get("RoadClass"),
                "speed": speed,
                "speed_valid": bool(reading.get("speed_valid")),
                "anomaly_reason": reading.get("anomaly_reason"),
                "color": color,
                "level": level,
                "level_name": reading.get("level_name") or "",
                "levels": levels,
                "vd_levels": entry.get("vd_levels"),
                "links": entry.get("links") or {},
                "dir_links": dir_links or [],
                "dir_mismatch": dir_mismatch,
                "status": entry.get("status"),
                "collect_time": entry.get("collect_time"),
                "on_route": on_route,
            })

            if not on_route:
                continue
            hits.append({
                "vdid": vdid,
                "s": s,
                "dir_links": dir_links or [],
                "color": color,
                "level": level,
                "levels": levels,
                "speed": speed,
                "collect_time": entry.get("collect_time"),
            })
    coverage = _coverage_from_hits(path, cum, turns, leg_bounds, hits)
    return points, coverage


def _coverage_from_hits(path: List[List[float]], cum: List[float], turns: List[float],
                        leg_bounds: List[float], hits: List[Dict[str, Any]]) -> List[Dict[str, Any]]:
    """依偵測器沿線位置 (hit["s"]) 建立覆蓋段：前後各延伸 500m、遇轉彎截斷；
    相鄰偵測器範圍重疊時，於兩偵測器中點平分，各自只負責自己那一半。"""
    if len(path) < 2 or not cum:
        return []
    total = cum[-1]
    items: List[List[Any]] = []
    for h in sorted(hits, key=lambda x: x["s"]):
        s = float(h["s"])
        back_turn = max([t for t in turns if t <= s], default=0.0)
        fwd_turn = min([t for t in turns if t >= s], default=total)
        items.append([h, max(0.0, min(EXTEND_M, s - back_turn)), max(0.0, min(EXTEND_M, fwd_turn - s))])
    for i in range(len(items) - 1):
        s0 = float(items[i][0]["s"])
        s1 = float(items[i + 1][0]["s"])
        if s0 + items[i][2] > s1 - items[i + 1][1]:
            mid = (s0 + s1) / 2.0
            items[i][2] = max(0.0, min(items[i][2], mid - s0))
            items[i + 1][1] = max(0.0, min(items[i + 1][1], s1 - mid))

    out: List[Dict[str, Any]] = []
    for h, back_len, fwd_len in items:
        s = float(h["s"])
        seg = _slice_path(path, cum, s - back_len, s + fwd_len)
        if len(seg) < 2:
            continue
        leg_index = max(0, len(leg_bounds) - 1)
        for i, b in enumerate(leg_bounds):
            if s <= b:
                leg_index = i
                break
        cov = {k: v for k, v in h.items() if k != "s"}
        cov.update({
            "leg_index": leg_index,
            "s_m": round(s, 1),
            "path": seg,
            "back_m": round(back_len, 1),
            "fwd_m": round(fwd_len, 1),
        })
        out.append(cov)
    return out

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
DEFAULT_BASE_SPEED_MPS = 60.0 / 3.6   # 無路名資訊時的基準速度（市區道路 60 km/h）
FALLBACK_DETOUR_FACTOR = 1.3          # OSRM 無法使用時，直線距離的道路彎曲折減率
TIME_FLOOR_KMH = 8.0                  # 偵測器車速計時下限（低於此值仍判壅塞，但以此速度計時）


def _step_speed_mps(step: Dict[str, Any]) -> float:
    """OSRM step 的基準計時速度：min(路網速度, 路名分類速限)。"""
    s_dist = float(step.get("distance") or 0.0)
    s_dur = float(step.get("duration") or 0.0)
    rtype = classify_street(step.get("name", ""), step.get("ref", ""))
    cap_mps = ROAD_RULES[rtype]["speed_cap_kmh"] / 3.6
    orig_mps = (s_dist / s_dur) if s_dur > 0 else cap_mps * 0.8
    eff = min(orig_mps, cap_mps)
    return eff if eff > 0 else cap_mps


def _normalize_speed_segments(segs: List[List[float]], total: float) -> List[List[float]]:
    """排序並補齊縫隙（沿用前一段速度），確保 [0, total] 全段都有基準速度。"""
    clean: List[List[float]] = []
    for a, b, v in (segs or []):
        a, b, v = float(a), min(float(b), total), float(v)
        if b > a and v > 0 and a < total:
            clean.append([a, b, v])
    clean.sort(key=lambda x: x[0])
    out: List[List[float]] = []
    for a, b, v in clean:
        if out and a < out[-1][1]:
            a = out[-1][1]
            if b <= a:
                continue
        if out and a > out[-1][1]:
            out[-1][1] = a
        out.append([a, b, v])
    if not out:
        return [[0.0, total, DEFAULT_BASE_SPEED_MPS]] if total > 0 else []
    out[0][0] = 0.0
    out[-1][1] = total
    return out


def base_time_s(segs: List[List[float]], d0: float, d1: float) -> float:
    """里程 [d0, d1] 依基準速度剖面（路名速限）的行駛秒數。"""
    if d1 <= d0:
        return 0.0
    if not segs:
        return (d1 - d0) / DEFAULT_BASE_SPEED_MPS
    t = 0.0
    for a, b, v in segs:
        if b <= d0:
            continue
        if a >= d1:
            break
        t += (min(b, d1) - max(a, d0)) / v
    if d0 < segs[0][0]:
        t += (min(d1, segs[0][0]) - d0) / segs[0][2]
    if d1 > segs[-1][1]:
        t += (d1 - max(d0, segs[-1][1])) / segs[-1][2]
    return t


def coverage_intervals(coverage: List[Dict[str, Any]]) -> List[List[float]]:
    """覆蓋段 → 不重疊的 [a_m, b_m, speed_kmh]（只取有有效車速者）。"""
    ivs: List[List[float]] = []
    for c in coverage or []:
        try:
            spd = float(c.get("speed") or 0.0)
            s = float(c.get("s_m"))
            a = s - float(c.get("back_m") or 0.0)
            b = s + float(c.get("fwd_m") or 0.0)
        except (TypeError, ValueError):
            continue
        if spd > 0 and b > a:
            ivs.append([a, b, spd])
    ivs.sort(key=lambda x: x[0])
    out: List[List[float]] = []
    for a, b, v in ivs:
        if out and a < out[-1][1]:
            a = out[-1][1]
        if b > a:
            out.append([a, b, v])
    return out


def travel_time_s(segs: List[List[float]], ivs: List[List[float]], d0: float, d1: float) -> float:
    """里程 [d0, d1] 行駛秒數：未覆蓋部分用路名基準速度，偵測器覆蓋部分改用偵測器車速（下限 8 km/h）。"""
    t = base_time_s(segs, d0, d1)
    for a, b, v in ivs:
        aa = max(a, d0)
        bb = min(b, d1)
        if bb <= aa:
            continue
        t += (bb - aa) / (max(float(v), TIME_FLOOR_KMH) / 3.6) - base_time_s(segs, aa, bb)
    return t


def profile_from_osrm_route(route: Dict[str, Any]) -> Dict[str, Any]:
    """由 OSRM route（steps=true）建立路線剖面：
    path/cum（幾何與累積里程）、turn_boundaries（轉彎點）、leg_boundaries（各 leg 終點）、
    road_segments（[起m, 終m, 路名]）、speed_segments（[起m, 終m, 基準速度 m/s]）、leg_streets（各 leg 路名）。"""
    path: List[List[float]] = []
    cum: List[float] = [0.0]
    turns: List[float] = []
    leg_bounds: List[float] = []
    roads: List[List[Any]] = []
    speeds: List[List[float]] = []
    leg_streets: List[List[str]] = []

    def _append(lat: float, lng: float):
        if path:
            prev = path[-1]
            if abs(prev[0] - lat) < 1e-9 and abs(prev[1] - lng) < 1e-9:
                return
            cum.append(cum[-1] + haversine_distance_meters(prev[0], prev[1], lat, lng))
        path.append([lat, lng])

    legs = route.get("legs", []) or []
    for leg in legs:
        names: List[str] = []
        for step in leg.get("steps", []) or []:
            st_name = step.get("name", "") or ""
            st_ref = step.get("ref", "") or ""
            if st_name and st_name not in names:
                names.append(st_name)
            seg_start = cum[-1]
            if path and is_turn_step(step):
                turns.append(cum[-1])
            for pt in (step.get("geometry") or {}).get("coordinates", []):
                _append(pt[1], pt[0])
            seg_end = cum[-1]
            if seg_end > seg_start:
                seg_name = st_name or st_ref
                if seg_name:
                    roads.append([seg_start, seg_end, seg_name])
                speeds.append([seg_start, seg_end, _step_speed_mps(step)])
        leg_bounds.append(cum[-1])
        leg_streets.append(names)

    if len(path) < 2:
        # step 幾何缺失：改用整體幾何，速度取 OSRM 平均
        path, cum = [], [0.0]
        for c in (route.get("geometry") or {}).get("coordinates", []):
            _append(c[1], c[0])
        total = cum[-1]
        dur = float(route.get("duration") or 0.0)
        speeds = [[0.0, total, (total / dur) if dur > 0 and total > 0 else DEFAULT_BASE_SPEED_MPS]]
        turns, roads = [], []
        leg_bounds = [total] * max(1, len(legs))

    total = cum[-1] if cum else 0.0
    return {
        "path": path,
        "cum": cum,
        "turn_boundaries": turns,
        "leg_boundaries": leg_bounds,
        "road_segments": roads,
        "speed_segments": _normalize_speed_segments(speeds, total),
        "leg_streets": leg_streets,
    }


def _route_result(prof: Dict[str, Any], geometry: List[List[float]], detour: float = 1.0) -> Dict[str, Any]:
    """由剖面整理各 leg 距離與基準時間（路名速限）。"""
    segs = prof["speed_segments"]
    legs_result = []
    prev = 0.0
    total_dist_m = 0.0
    for i, end in enumerate(prof["leg_boundaries"]):
        names = prof["leg_streets"][i] if i < len(prof["leg_streets"]) else []
        dist_m = max(0.0, end - prev) * detour
        total_dist_m += dist_m
        legs_result.append({
            "leg_index": i,
            "from_seq": i,
            "to_seq": i + 1,
            "distance_km": round(dist_m / 1000.0, 2),
            "duration_min": round(base_time_s(segs, prev, end) / 60.0, 1),
            "main_street": " → ".join(names[:3]) if names else "一般街道",
            "traffic_status": "GREEN",
        })
        prev = end
    total = prof["cum"][-1] if prof["cum"] else 0.0
    return {
        "geometry": geometry,
        "total_distance_km": round(total_dist_m / 1000.0, 2),
        "total_duration_min": round(base_time_s(segs, 0.0, total) / 60.0, 1),
        "legs": legs_result,
        "profile": prof,
    }


def compute_route_with_road_speed(ordered_coords: List[List[float]]) -> Dict[str, Any]:
    if len(ordered_coords) < 2:
        return {
            "geometry": [],
            "total_distance_km": 0,
            "total_duration_min": 0,
            "legs": [],
            "profile": {"path": [], "cum": [], "turn_boundaries": [], "leg_boundaries": [],
                        "road_segments": [], "speed_segments": [], "leg_streets": []}
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
                return _route_result(profile_from_osrm_route(route_info), latlng_path)
    except Exception as e:
        print(f"OSRM Route request error: {e}")

    # Fallback：直線距離 × 道路彎曲折減率，以 60 km/h 估算
    fb_path = [[c[1], c[0]] for c in ordered_coords]
    fb_cum = [0.0]
    for i in range(1, len(fb_path)):
        a, b = fb_path[i - 1], fb_path[i]
        fb_cum.append(fb_cum[-1] + haversine_distance_meters(a[0], a[1], b[0], b[1]))
    total = fb_cum[-1]
    prof = {
        "path": fb_path,
        "cum": fb_cum,
        "turn_boundaries": [],
        "leg_boundaries": fb_cum[1:],
        "road_segments": [],
        "speed_segments": _normalize_speed_segments(
            [[0.0, total, DEFAULT_BASE_SPEED_MPS / FALLBACK_DETOUR_FACTOR]], total),
        "leg_streets": [["市區主要道路"] for _ in range(len(fb_path) - 1)],
    }
    return _route_result(prof, fb_path, detour=FALLBACK_DETOUR_FACTOR)

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
            "route_meta": {"cum": [], "turn_boundaries": [], "leg_boundaries": [],
                           "road_segments": [], "speed_segments": []}
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
            "leg_boundaries": route_calc["profile"]["leg_boundaries"],
            "road_segments": route_calc["profile"]["road_segments"],
            "speed_segments": route_calc["profile"]["speed_segments"],
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
REROUTE_ALT_MAX = 3            # 每個改道事件最多保留的候選路徑數
REROUTE_MIN_SAVED_MIN = 0.5    # 只保留預估節省大於 0.5 分鐘的改道備選
REROUTE_MIN_DEVIATION_M = 5.0  # 備選與主線偏離小於此值視為假備選丟棄
REROUTE_SCOPE_LEVELS = 2       # 改道點/回歸點向外搜尋的階數（第 0 階為最近轉彎點，有結果即停止）
REROUTE_SCOPE_STEP_M = 200.0   # 轉彎稀疏時的固定擴展步進 (公尺)
REROUTE_COMBINE_GAP_M = 300.0  # 連續壅塞段間隔 ≤ 此值整合為 COMBINED 事件
REROUTE_ANCHOR_M = 15.0        # 錨點：P1 前、P2 後各退 15m，確保位於進入/離開的道路上
REROUTE_BEARING_TOLS = (45, 90)  # 錨點方向容許角度（度），依序放寬，最後不限方向
REROUTE_VIA_RATIO = 0.3        # 側推距離 = P1→P2 長度 × 比例，並限制於下列上下限
REROUTE_VIA_MIN_M = 200.0
REROUTE_VIA_MAX_M = 800.0
REROUTE_OVERLAP_M = 30.0       # 替代路距被取代原路段 ≤ 此值視為仍走在原路上
REROUTE_REVERT_DIST_M = 40.0   # 與已被取代路段距離 ≤ 此值的取樣點視為重疊
REROUTE_REVERT_RATIO = 0.70    # 重疊取樣點達此比例視為繞回已被取代的原路


def _vd_path_reading(vd: Dict[str, Any], path: List[List[float]], cum: List[float],
                     s: float) -> Optional[Dict[str, Any]]:
    """前端傳入的 VD 在某路徑里程 s 處、依該路徑行進方向的讀值；對向偵測器回傳 None。"""
    links = vd.get("links") or {}
    if not links:
        # 無方向資訊（舊資料）：沿用前端傳入的有效速度
        spd = vd.get("speed")
        valid = isinstance(spd, (int, float)) and spd > 0 and not vd.get("anomaly_reason")
        return {
            "speed": float(spd) if valid else None,
            "speed_valid": bool(valid),
            "color": vd.get("color") or "GRAY",
            "anomaly_reason": vd.get("anomaly_reason") or (None if valid else "無資料"),
        }
    ids = tdx_client.links_for_heading(links, _bearing_at(path, cum, s))
    if ids is None:
        return None
    entry = {"status": vd.get("status"), "links": links, "vd_levels": vd.get("vd_levels")}
    return tdx_client.direction_reading(entry, ids, vd.get("sim_level"))


def _path_coverage(prof: Dict[str, Any], vds: List[Dict[str, Any]]) -> List[Dict[str, Any]]:
    """任一路徑（如替代路）上的偵測器覆蓋段，與主線相同規則：
    距離 ≤ 50m、路名相容、同行進方向、遇轉彎截斷、相鄰中點平分。
    每段含 speed（有效車速，無效為 None）、color、anomaly_reason。"""
    path, cum = prof["path"], prof["cum"]
    roads = prof.get("road_segments") or []
    hits: List[Dict[str, Any]] = []
    for vd in vds:
        try:
            lat_v = float(vd["lat"])
            lon_v = float(vd["lon"])
        except (KeyError, TypeError, ValueError):
            continue
        off, s = _project_point_to_path(lat_v, lon_v, path, cum)
        if off > COVERAGE_THRESHOLD_M:
            continue
        if roads and not _road_name_compatible(vd.get("road", ""), _road_name_at(roads, s)):
            continue
        reading = _vd_path_reading(vd, path, cum, s)
        if reading is None:
            continue  # 對向偵測器，不屬於此路徑行進方向
        valid = bool(reading.get("speed_valid"))
        hits.append({
            "vdid": vd.get("vdid", ""),
            "s": s,
            "speed": reading["speed"] if valid else None,
            "color": reading.get("color") or "GRAY",
            "anomaly_reason": None if valid else (reading.get("anomaly_reason") or "無資料"),
        })
    return _coverage_from_hits(path, cum, prof.get("turn_boundaries") or [], [], hits)


def _max_deviation_m(alt_path: List[List[float]], alt_cum: List[float],
                     main_path: List[List[float]], main_cum: List[float],
                     ignore_m: float = 30.0) -> Tuple[float, List[float], List[float]]:
    """候選路徑相對主線的最大偏離距離，以及 (第一個偏離點, 最後一個偏離點)。

    忽略起訖 ignore_m 公尺，避免端點吸附到鄰近道路造成的假訊號。
    """
    max_off = 0.0
    exit_at: List[float] = []
    rejoin_at: List[float] = []
    total = alt_cum[-1] if alt_cum else 0.0
    step = max(1, len(alt_path) // 60)
    for i in range(0, len(alt_path), step):
        d = alt_cum[i] if i < len(alt_cum) else total
        if d < ignore_m or d > total - ignore_m:
            continue
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


def get_via_point(p1: List[float], p2: List[float],
                  offset_meters: float = 600.0, side: str = "right") -> List[float]:
    """
    p1: [lat1, lon1], p2: [lat2, lon2]
    向行進方向左側或右側垂直偏移 offset_meters，並用 OSRM /nearest 吸附到最近道路。
    回傳 [via_lat, via_lon]
    """
    lat1, lon1 = p1[0], p1[1]
    lat2, lon2 = p2[0], p2[1]
    mid_lat = (lat1 + lat2) / 2.0
    mid_lon = (lon1 + lon2) / 2.0

    dlat = lat2 - lat1
    dlon = lon2 - lon1

    # 經緯度尺度換算 (台灣緯度約 24~25 度：1度緯度約 111,000m，1度經度約 102,000m)
    scale_y = 111000.0
    scale_x = 102000.0

    dx_m = dlon * scale_x
    dy_m = dlat * scale_y
    length_m = math.hypot(dx_m, dy_m)
    if length_m < 1.0:
        return [lat1, lon1]

    ux = dx_m / length_m
    uy = dy_m / length_m

    if side == "right":
        perp_x, perp_y = uy, -ux
    else:
        perp_x, perp_y = -uy, ux

    offset_lon = mid_lon + (perp_x * offset_meters) / scale_x
    offset_lat = mid_lat + (perp_y * offset_meters) / scale_y

    # 吸附至最近的道路節點 (OSRM nearest)
    try:
        url = f"{OSRM_BASE_URL}/nearest/v1/driving/{offset_lon:.6f},{offset_lat:.6f}"
        resp = requests.get(url, headers=HEADERS, timeout=4)
        if resp.status_code == 200:
            data = resp.json()
            if data.get("code") == "Ok" and data.get("waypoints"):
                loc = data["waypoints"][0]["location"]  # [lon, lat]
                return [loc[1], loc[0]]
    except Exception:
        pass

    return [offset_lat, offset_lon]


def _bearing_at(path: List[List[float]], cum: List[float], s: float) -> Optional[float]:
    """計算主線路徑在累積里程 s 處的前進方向角 (0~360度，正北為0，順時針)。"""
    if len(path) < 2 or len(cum) < 2:
        return None
    p_curr = _path_point_at(path, cum, max(0.0, s))
    p_next = _path_point_at(path, cum, min(cum[-1], s + 25.0))
    lat1, lon1 = p_curr
    lat2, lon2 = p_next
    if abs(lat2 - lat1) < 1e-6 and abs(lon2 - lon1) < 1e-6:
        p_prev = _path_point_at(path, cum, max(0.0, s - 25.0))
        lat1, lon1 = p_prev
        lat2, lon2 = p_curr
    if abs(lat2 - lat1) < 1e-6 and abs(lon2 - lon1) < 1e-6:
        return None
    y = math.sin(math.radians(lon2 - lon1)) * math.cos(math.radians(lat2))
    x = (math.cos(math.radians(lat1)) * math.sin(math.radians(lat2)) -
         math.sin(math.radians(lat1)) * math.cos(math.radians(lat2)) * math.cos(math.radians(lon2 - lon1)))
    bearing = (math.degrees(math.atan2(y, x)) + 360.0) % 360.0
    return round(bearing, 1)


def _has_uturn(route: Dict[str, Any]) -> bool:
    """檢查 route 中的 maneuvers 是否包含迴轉 (U-turn) 動作，防止走進死巷調頭。"""
    for leg in route.get("legs", []) or []:
        for step in leg.get("steps", []) or []:
            maneuver = step.get("maneuver") or {}
            mtype = str(maneuver.get("type", "")).lower()
            mmod = str(maneuver.get("modifier", "")).lower()
            if mtype == "uturn" or mmod == "uturn":
                return True
    return False


def _has_retrace_turn(path: List[List[float]]) -> bool:
    """方案一：幾何原路折返檢測（檢測是否原路進出死巷或在同條路反向折返）。"""
    if len(path) < 4:
        return False

    cum = [0.0]
    for i in range(1, len(path)):
        cum.append(cum[-1] + haversine_distance_meters(path[i - 1][0], path[i - 1][1], path[i][0], path[i][1]))
    total_m = cum[-1]
    if total_m < 60.0:
        return False

    # 1. 均勻取樣 (每 15 公尺一個點)
    step_m = 15.0
    cur_s = 0.0
    samples: List[Tuple[float, float, float]] = []
    while cur_s <= total_m:
        pt = _path_point_at(path, cum, cur_s)
        samples.append((pt[0], pt[1], cur_s))
        cur_s += step_m

    if len(samples) < 4:
        return False

    # 2. 計算各取樣點前進單位向量
    lat_mid = samples[len(samples) // 2][0]
    scale_y = 111000.0
    scale_x = 111000.0 * math.cos(math.radians(lat_mid))

    vectors: List[Tuple[float, float]] = []
    for i in range(len(samples) - 1):
        dy = (samples[i + 1][0] - samples[i][0]) * scale_y
        dx = (samples[i + 1][1] - samples[i][1]) * scale_x
        norm = math.hypot(dx, dy)
        if norm > 1e-4:
            vectors.append((dx / norm, dy / norm))
        else:
            vectors.append((0.0, 0.0))

    # 3. 雙重迴圈檢驗：空間距離 <= 25m 但沿路里程差距 >= 50m 且方向相反 (dot < -0.70)
    retrace_len = 0.0
    for i in range(len(vectors)):
        vx_i, vy_i = vectors[i]
        if vx_i == 0.0 and vy_i == 0.0:
            continue
        p_i = samples[i]
        for j in range(i + 3, len(vectors)):
            p_j = samples[j]
            if (p_j[2] - p_i[2]) < 50.0:
                continue
            dist = haversine_distance_meters(p_i[0], p_i[1], p_j[0], p_j[1])
            if dist <= 25.0:
                vx_j, vy_j = vectors[j]
                dot = vx_i * vx_j + vy_i * vy_j
                if dot < -0.70:  # 反向夾角 > 135度，代表在同條路上朝相反方向開
                    retrace_len += step_m
                    if retrace_len >= 30.0:
                        return True
                    break

    return False


def _heading_between(path: List[List[float]], cum: List[float], s0: float, s1: float) -> Optional[float]:
    """路徑上里程 s0 → s1 的行進方位角（度）；兩點重合時回傳 None。"""
    if len(path) < 2 or not cum:
        return None
    a = _path_point_at(path, cum, max(0.0, s0))
    b = _path_point_at(path, cum, min(cum[-1], s1))
    if abs(a[0] - b[0]) < 1e-7 and abs(a[1] - b[1]) < 1e-7:
        return None
    return _bearing_deg(a[0], a[1], b[0], b[1])


def _osrm_route_try(points: List[List[float]], headings: List[Optional[float]], extra: str = "") -> List[Dict[str, Any]]:
    """向 OSRM 索取路線（points 為 [lat, lon]）；有方向限制時依 ±45°、±90°、不限方向逐步放寬。"""
    coord_str = ";".join(f"{pt[1]:.6f},{pt[0]:.6f}" for pt in points)
    base_url = (f"{OSRM_BASE_URL}/route/v1/driving/{coord_str}"
                f"?overview=full&geometries=geojson&steps=true&continue_straight=true{extra}")
    tries: List[str] = []
    if any(h is not None for h in headings):
        for tol in REROUTE_BEARING_TOLS:
            tries.append("&bearings=" + ";".join("" if h is None else f"{h:.0f},{tol}" for h in headings))
    tries.append("")
    for bearing_arg in tries:
        try:
            resp = requests.get(base_url + bearing_arg, headers=HEADERS, timeout=6)
        except Exception:
            continue
        if resp.status_code == 200:
            routes = resp.json().get("routes") or []
            if routes:
                return routes
    return []


def _sub_path(path: List[List[float]], cum: List[float], d0: float, d1: float) -> Tuple[List[List[float]], List[float], float]:
    """取出里程 [d0, d1] 的子路徑與其相對里程，回傳 (path, cum, 起點里程)。"""
    d0 = max(0.0, d0)
    sub = _slice_path(path, cum, d0, d1)
    sub_cum = [0.0]
    for i in range(1, len(sub)):
        sub_cum.append(sub_cum[-1] + haversine_distance_meters(sub[i - 1][0], sub[i - 1][1], sub[i][0], sub[i][1]))
    return sub, sub_cum, d0


def _alt_estimate_s(alt_prof: Dict[str, Any], path: List[List[float]], cum: List[float],
                    main_ivs: List[List[float]], win_a: float, win_b: float) -> float:
    """替代路預估時間：路名基準速度；但仍走在被取代原路段 [win_a, win_b]（距離 ≤30m）上的部分，
    改用原路該處的偵測器車速（同一條路，與原路計時一致，不以速限計）。"""
    segs = alt_prof["speed_segments"]
    a_path, a_cum = alt_prof["path"], alt_prof["cum"]
    total = a_cum[-1] if a_cum else 0.0
    t = base_time_s(segs, 0.0, total)
    cong_ivs = [iv for iv in main_ivs if iv[1] > win_a and iv[0] < win_b]
    if not cong_ivs or total <= 0:
        return t
    sub, sub_cum, off0 = _sub_path(path, cum, win_a - REROUTE_OVERLAP_M, win_b + REROUTE_OVERLAP_M)
    if len(sub) < 2:
        return t
    step = 10.0
    d = 0.0
    while d < total:
        d_end = min(total, d + step)
        mid = _path_point_at(a_path, a_cum, (d + d_end) / 2.0)
        off, s_local = _project_point_to_path(mid[0], mid[1], sub, sub_cum)
        s = s_local + off0
        if off <= REROUTE_OVERLAP_M and win_a <= s <= win_b:
            for a, b, v in cong_ivs:
                if a <= s <= b:
                    t += (d_end - d) / (max(float(v), TIME_FLOOR_KMH) / 3.6) - base_time_s(segs, d, d_end)
                    break
        d = d_end
    return t


def _reverts_to_replaced(alt_path: List[List[float]], replaced: List[List[float]]) -> bool:
    """替代路中段（20%~80%）的取樣點有 70% 以上落在已被取代路段 40m 內 → 視為繞回原路。"""
    if len(alt_path) < 2 or len(replaced) < 2:
        return False
    n = len(alt_path)
    samples = alt_path[max(1, int(n * 0.2)):min(n - 1, int(n * 0.8))]
    if not samples:
        return False
    rep_cum = [0.0]
    for i in range(1, len(replaced)):
        rep_cum.append(rep_cum[-1] + haversine_distance_meters(replaced[i - 1][0], replaced[i - 1][1],
                                                               replaced[i][0], replaced[i][1]))
    close = 0
    for pt in samples:
        off, _ = _project_point_to_path(pt[0], pt[1], replaced, rep_cum)
        if off <= REROUTE_REVERT_DIST_M:
            close += 1
    return close / len(samples) >= REROUTE_REVERT_RATIO


def _reroute_candidates(path: List[List[float]], cum: List[float],
                        main_segs: List[List[float]], main_ivs: List[List[float]],
                        vds: List[Dict[str, Any]],
                        p1: float, p2: float, leg_start: float, leg_end: float,
                        main_total_s: float,
                        replaced_segments: List[List[List[float]]]) -> Tuple[List[Dict[str, Any]], Dict[str, Any]]:
    """索取改道候選並計時比較。

    候選來源：排除國道、OSRM 原生替代路線、左右側推點（距離依 P1→P2 長度調整）。
    起訖錨點：P1 前 15m（方向＝進入方向）、P2 後 15m（方向＝離開方向），避免從壅塞路出發或回歸時掉頭。
    計時：原路＝路名基準速度＋覆蓋段偵測器車速；替代路預估＝路名基準速度，
    但仍走在被取代原路段上的部分用原路偵測器車速。只回傳預估節省 > REROUTE_MIN_SAVED_MIN 的候選。"""
    stats: Dict[str, Any] = {
        "osrm_routes": 0,
        "nonfake_alts": 0,
        "reverted_alts": 0,
        "best_nonfake_saved_min": None,
    }
    # 錨點：P1 前、P2 後各退 15m，並取該處主線行進方向
    s1 = max(leg_start, p1 - REROUTE_ANCHOR_M)
    s2 = min(leg_end, p2 + REROUTE_ANCHOR_M)
    pt1 = _path_point_at(path, cum, s1)
    pt2 = _path_point_at(path, cum, s2)
    h1 = _heading_between(path, cum, s1 - 20.0, s1)
    if h1 is None:
        h1 = _heading_between(path, cum, s1, s1 + 20.0)
    h2 = _heading_between(path, cum, s2, s2 + 20.0)
    if h2 is None:
        h2 = _heading_between(path, cum, s2 - 20.0, s2)

    raw_routes: List[Dict[str, Any]] = []
    # 策略 1：排除國道（高架/國道壅塞時引導走平面）
    raw_routes.extend(_osrm_route_try([pt1, pt2], [h1, h2], "&exclude=motorway")[:1])
    # 策略 2：OSRM 原生替代路線（最短路＋最多 2 條替代）
    raw_routes.extend(_osrm_route_try([pt1, pt2], [h1, h2], "&alternatives=2"))
    # 策略 3：左右側推點，距離依 P1→P2 長度調整
    via_offset = min(REROUTE_VIA_MAX_M, max(REROUTE_VIA_MIN_M, REROUTE_VIA_RATIO * (s2 - s1)))
    for side in ("right", "left"):
        via_pt = get_via_point(pt1, pt2, offset_meters=via_offset, side=side)
        d_v1 = haversine_distance_meters(pt1[0], pt1[1], via_pt[0], via_pt[1])
        d_v2 = haversine_distance_meters(pt2[0], pt2[1], via_pt[0], via_pt[1])
        if d_v1 < 30.0 or d_v2 < 30.0:
            continue
        raw_routes.extend(_osrm_route_try([pt1, via_pt, pt2], [h1, None, h2])[:1])

    if not raw_routes:
        return [], stats

    stats["osrm_routes"] = len(raw_routes)
    # 原路 P1→P2：未覆蓋用路名基準速度，覆蓋段用偵測器車速
    main_p1p2_s = travel_time_s(main_segs, main_ivs, s1, s2)
    base_p1p2_s = base_time_s(main_segs, s1, s2)
    alts: List[Dict[str, Any]] = []

    seen_geoms = set()
    for rt in raw_routes:
        # 防迴轉檢查：包含 U-turn 動作直接剃除
        if _has_uturn(rt):
            continue

        alt_prof = profile_from_osrm_route(rt)
        alt_path, alt_cum = alt_prof["path"], alt_prof["cum"]
        if len(alt_path) < 2:
            continue

        # 方案一：幾何原路折返檢測（進出死巷或在同條路反向調頭直接剃除）
        if _has_retrace_turn(alt_path):
            continue

        # 幾何重複過濾
        mid_idx = len(alt_path) // 2
        geom_key = (len(alt_path), round(alt_path[mid_idx][0], 4), round(alt_path[mid_idx][1], 4))
        if geom_key in seen_geoms:
            continue
        seen_geoms.add(geom_key)

        alt_total_m = alt_cum[-1]
        if alt_total_m <= 0:
            continue

        # 幾何過濾：與主線幾乎重疊（假備選）才丟棄
        max_off, exit_at, rejoin_at = _max_deviation_m(alt_path, alt_cum, path, cum)
        if max_off <= REROUTE_MIN_DEVIATION_M:
            continue
        stats["nonfake_alts"] += 1

        # 套用改道後：繞回已被取代的原路段者剔除
        if any(_reverts_to_replaced(alt_path, seg) for seg in replaced_segments):
            stats["reverted_alts"] += 1
            continue

        # 1. 預估省時：替代路以與原路相同的路名基準速度計時；仍走在原路段上的部分用原路偵測器車速
        alt_segs = alt_prof["speed_segments"]
        alt_est_s = _alt_estimate_s(alt_prof, path, cum, main_ivs, s1, s2)
        saved_min = (main_p1p2_s - alt_est_s) / 60.0

        best_nonfake = stats["best_nonfake_saved_min"]
        if best_nonfake is None or saved_min > best_nonfake:
            stats["best_nonfake_saved_min"] = saved_min
        if saved_min <= REROUTE_MIN_SAVED_MIN:
            continue

        # 2. 即時省時：替代路上的偵測器覆蓋段改用偵測器車速（與主線同一套覆蓋規則）
        alt_covs = _path_coverage(alt_prof, vds)
        valid_covs = [c for c in alt_covs if c.get("speed")]
        anomaly_covs = [c for c in alt_covs if not c.get("speed")]
        has_vd_coverage = bool(valid_covs)
        rt_saved_min: Optional[float] = None
        rt_color = "GRAY"
        anomaly_tag = None

        if has_vd_coverage:
            alt_rt_s = travel_time_s(alt_segs, coverage_intervals(valid_covs), 0.0, alt_total_m)
            rt_saved_min = (main_p1p2_s - alt_rt_s) / 60.0
            # 顏色嚴重度僅依據有效偵測器：RED > ORANGE > GREEN
            for c_name in ("RED", "ORANGE", "GREEN"):
                if any(c.get("color") == c_name for c in valid_covs):
                    rt_color = c_name
                    break
        elif anomaly_covs:
            # 只覆蓋到異常偵測器：標註原因（呈現類似無覆蓋之灰色狀態）
            reasons = [str(c.get("anomaly_reason") or "") for c in anomaly_covs]
            if any("車速" in r for r in reasons):
                anomaly_tag = "車速異常"
            elif any("無" in r for r in reasons):
                anomaly_tag = "無資料"
            elif any("設備" in r for r in reasons):
                anomaly_tag = "設備異常"
            else:
                anomaly_tag = "資料異常"

        alts.append({
            "seg_polyline": alt_path,
            # 替代路自身剖面（相對替代路起點的里程），套用改道時拼接回主線剖面
            "seg_speed_segments": alt_segs,
            "seg_road_segments": alt_prof["road_segments"],
            "seg_turns": alt_prof["turn_boundaries"],
            "p1_m": round(s1, 1),
            "p2_m": round(s2, 1),
            "leg_start_m": round(leg_start, 1),
            "leg_end_m": round(leg_end, 1),
            "main_p1p2_min": round(main_p1p2_s / 60.0, 1),
            "base_p1p2_min": round(base_p1p2_s / 60.0, 1),
            "alt_p1p2_min": round(alt_est_s / 60.0, 1),
            "eta_min": round((main_total_s - main_p1p2_s + alt_est_s) / 60.0, 1),
            "saved_min": round(saved_min, 1),
            "distance_km": round(alt_total_m / 1000.0, 2),
            "exit_at": exit_at,
            "rejoin_at": rejoin_at,
            "has_vd_coverage": has_vd_coverage,
            "realtime_saved_min": round(rt_saved_min, 1) if rt_saved_min is not None else None,
            "realtime_color": rt_color,
            "anomaly_tag": anomaly_tag,
        })
    alts.sort(key=lambda a: (-a["saved_min"], a["eta_min"]))
    return alts, stats


class RouteAltVd(BaseModel):
    vdid: str = ""
    lat: float
    lon: float
    road: Optional[str] = ""
    speed: Optional[float] = None
    color: Optional[str] = "GRAY"
    anomaly_reason: Optional[str] = None
    status: Optional[Any] = None
    links: Dict[str, Any] = {}          # {LinkID: {bearing, lanes, levels}}，依方向取值用
    vd_levels: Optional[List[Dict[str, Any]]] = None
    sim_level: Optional[int] = None     # 前端模擬壅塞等級


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
    speed_segments: List[List[float]] = []   # 主線基準速度剖面 [起m, 終m, m/s]
    road_segments: List[List[Any]] = []
    replaced_segments: List[List[List[float]]] = []  # 已套用改道所取代的原路段（避免建議繞回）


def _are_routes_duplicate(alt_a: Dict[str, Any], alt_b: Dict[str, Any]) -> bool:
    """比對兩條備選路線是否為實質相同的替代道路且節省時間相近。"""
    # 1. 時間節省比對：預估節省時間差異若超過 0.5 分鐘，視為不同改道策略
    s_a = float(alt_a.get("saved_min") or 0.0)
    s_b = float(alt_b.get("saved_min") or 0.0)
    if abs(s_a - s_b) > 0.5:
        return False

    # 若兩者都有即時省時，比對即時省時差異
    rt_a = alt_a.get("realtime_saved_min")
    rt_b = alt_b.get("realtime_saved_min")
    if rt_a is not None and rt_b is not None:
        if abs(float(rt_a) - float(rt_b)) > 0.5:
            return False

    # 2. 距離長度比對：總里程差異超過 20% 視為不同路線
    d_a = float(alt_a.get("distance_km") or 0.0)
    d_b = float(alt_b.get("distance_km") or 0.0)
    if d_a > 0 and d_b > 0 and abs(d_a - d_b) / max(d_a, d_b) > 0.20:
        return False

    # 3. 幾何道路比對：比對中間取樣點與起迄點
    poly_a = alt_a.get("seg_polyline") or []
    poly_b = alt_b.get("seg_polyline") or []
    if len(poly_a) < 2 or len(poly_b) < 2:
        p1_a, p2_a = float(alt_a.get("p1_m") or 0.0), float(alt_a.get("p2_m") or 0.0)
        p1_b, p2_b = float(alt_b.get("p1_m") or 0.0), float(alt_b.get("p2_m") or 0.0)
        return abs(p1_a - p1_b) < 100.0 and abs(p2_a - p2_b) < 100.0

    # 取中間點經緯度比對距離
    mid_a = poly_a[len(poly_a) // 2]
    mid_b = poly_b[len(poly_b) // 2]
    dist_mid = haversine_distance_meters(mid_a[0], mid_a[1], mid_b[0], mid_b[1])
    if dist_mid > 70.0:
        return False

    # 起點與終點距離比對（改道切出點與切回點距離）
    dist_start = haversine_distance_meters(poly_a[0][0], poly_a[0][1], poly_b[0][0], poly_b[0][1])
    dist_end = haversine_distance_meters(poly_a[-1][0], poly_a[-1][1], poly_b[-1][0], poly_b[-1][1])
    if dist_start > 150.0 or dist_end > 150.0:
        return False

    return True


def _deduplicate_leg_reroutes(events: List[Dict[str, Any]]) -> List[Dict[str, Any]]:
    """在同一個 Leg 內，比對道路與時間節省，合併減少重複的改道事件。"""
    can_events = [e for e in events if e.get("state") == "CAN_REROUTE"]
    other_events = [e for e in events if e.get("state") != "CAN_REROUTE"]
    if len(can_events) <= 1:
        return events

    merged_can: List[Dict[str, Any]] = []
    for cur in can_events:
        cur_best = (cur.get("alternatives") or [{}])[0]
        duplicate_target = None
        for existing in merged_can:
            exist_best = (existing.get("alternatives") or [{}])[0]
            if _are_routes_duplicate(cur_best, exist_best):
                duplicate_target = existing
                break

        if duplicate_target is None:
            merged_can.append(cur)
        else:
            # 發生重複：優先保留 COMBINED 事件；若同類型則保留省時更長或涵蓋範圍更廣者
            target_best = (duplicate_target.get("alternatives") or [{}])[0]
            cur_is_comb = cur.get("event_type") == "COMBINED"
            tgt_is_comb = duplicate_target.get("event_type") == "COMBINED"

            should_replace = False
            if cur_is_comb and not tgt_is_comb:
                should_replace = True
            elif cur_is_comb == tgt_is_comb:
                cur_save = cur_best.get("realtime_saved_min") if cur_best.get("realtime_saved_min") is not None else cur_best.get("saved_min", 0.0)
                tgt_save = target_best.get("realtime_saved_min") if target_best.get("realtime_saved_min") is not None else target_best.get("saved_min", 0.0)
                if cur_save > tgt_save:
                    should_replace = True

            chosen = cur if should_replace else duplicate_target
            abandoned = duplicate_target if should_replace else cur

            # 把被合併事件中走「不同道路」的額外備選方案收納進來
            for extra_alt in abandoned.get("alternatives", []):
                if not any(_are_routes_duplicate(extra_alt, exist_alt) for exist_alt in chosen.get("alternatives", [])):
                    chosen["alternatives"].append(extra_alt)

            if should_replace:
                merged_can.remove(duplicate_target)
                merged_can.append(chosen)

    # 針對留下來的各事件內部，也過濾掉實質相同的備選方案
    for ev in merged_can:
        clean_alts: List[Dict[str, Any]] = []
        for alt in ev.get("alternatives", []):
            if not any(_are_routes_duplicate(alt, ca) for ca in clean_alts):
                clean_alts.append(alt)
        ev["alternatives"] = clean_alts
        if clean_alts:
            ev["best_saved_min"] = clean_alts[0].get("saved_min")

    return merged_can + other_events


@app.post("/api/route/alternatives")
def route_alternatives(req: RouteAlternativesRequest):
    """將每條 leg 的壅塞依間隔分群成多個事件，各別推算改道點 (P1)/回歸點 (P2)，
    只回傳節省 > 0.5 分鐘的候選與狀態 (CAN_REROUTE/CANNOT_REROUTE)。"""
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

    # 1) 收集每條 leg 的橘/紅壅塞區間（稍後依間隔分群為多個事件）
    by_leg: Dict[int, List[List[Any]]] = {}
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
        by_leg.setdefault(int(li), []).append([a, b, color])

    if not by_leg:
        return {"status": "success", "legs": []}

    # 分群：同時產生「單點」與「整合」雙層候選
    leg_events: Dict[int, List[Dict[str, Any]]] = {}
    for li, ivs in by_leg.items():
        ivs.sort(key=lambda x: x[0])
        # 1. 單點事件：每支壅塞偵測器自己的覆蓋段（覆蓋段已於中點平分，彼此不重疊）
        singles: List[Dict[str, Any]] = [
            {"a": a, "b": b, "worst": color, "event_type": "SINGLE"} for a, b, color in ivs
        ]

        slots: List[Dict[str, Any]] = list(singles)

        # 2. 整合事件：2 段以上連續壅塞（相接或間隔 ≤ 300m）串成一個大路段
        if len(singles) >= 2:
            comb_groups: List[List[Dict[str, Any]]] = []
            cur_group = [singles[0]]
            for nxt in singles[1:]:
                if nxt["a"] - cur_group[-1]["b"] <= REROUTE_COMBINE_GAP_M:
                    cur_group.append(nxt)
                else:
                    if len(cur_group) >= 2:
                        comb_groups.append(cur_group)
                    cur_group = [nxt]
            if len(cur_group) >= 2:
                comb_groups.append(cur_group)

            for grp in comb_groups:
                c_a = grp[0]["a"]
                c_b = max(g["b"] for g in grp)
                worst_c = "RED" if any(g["worst"] == "RED" for g in grp) else "ORANGE"
                slots.append({
                    "a": c_a,
                    "b": c_b,
                    "worst": worst_c,
                    "event_type": "COMBINED",
                    "tag": f"整合連續壅塞（涵蓋 {len(grp)} 處塞車）"
                })

        leg_events[li] = slots

    # 主線基準速度剖面（路名速限）；舊前端未提供時，退回各 leg 平均速度
    if req.speed_segments:
        main_segs = _normalize_speed_segments(req.speed_segments, total_m)
    else:
        fb_segs: List[List[float]] = []
        prev = 0.0
        for i, end in enumerate(leg_bounds or [total_m]):
            info = legs_info.get(i, {})
            base_min = float(info.get("base_duration_min") or info.get("duration_min") or 0.0)
            if end > prev:
                fb_segs.append([prev, end, (end - prev) / (base_min * 60.0) if base_min > 0 else DEFAULT_BASE_SPEED_MPS])
            prev = end
        main_segs = _normalize_speed_segments(fb_segs, total_m)
    # 主線偵測器覆蓋段（前端依行進方向/模擬推算的有效車速）
    main_ivs = coverage_intervals(req.coverage)
    main_total_s = sum(float(x.get("duration_min") or 0.0) * 60.0 for x in legs_info.values())
    if main_total_s <= 0:
        main_total_s = travel_time_s(main_segs, main_ivs, 0.0, total_m)

    out_legs: List[Dict[str, Any]] = []
    for li in sorted(leg_events.keys()):
        slots = leg_events[li]
        leg_start = leg_bounds[li - 1] if 0 < li < len(leg_bounds) else 0.0
        leg_end = leg_bounds[li] if li < len(leg_bounds) else total_m

        leg_items: List[Dict[str, Any]] = []
        for slot in slots:
            a, b = slot["a"], slot["b"]
            etype = slot.get("event_type", "SINGLE")
            etag = slot.get("tag", "")

            # 改道範圍：以本 Leg 邊界為限，確保不越站配送
            evt_start = leg_start
            evt_end = leg_end

            ev_code = "c" if etype == "COMBINED" else "s"
            base: Dict[str, Any] = {
                "leg_index": li,
                "event_id": f"{li}:{ev_code}:{round(a, 1)}",
                "event_type": etype,
                "tag": etag,
                "a_m": round(a, 1),
                "b_m": round(b, 1),
                "alternatives": [],
            }
            if evt_end - evt_start < REROUTE_MIN_WINDOW_M:
                leg_items.append({**base, "state": "CANNOT_REROUTE", "reason": "WINDOW_TOO_SHORT"})
                continue

            # 2) 候選改道點：壅塞前/後取更遠的轉彎；轉彎用盡後以固定步進往兩側外推
            turns_before = [t for t in turns if t <= a]
            turns_after = [t for t in turns if t >= b]

            def _cand_p1(k: int) -> float:
                if k < len(turns_before):
                    return turns_before[-1 - k]
                ref = turns_before[0] if turns_before else a
                return ref - REROUTE_SCOPE_STEP_M * (k - len(turns_before) + 1)

            def _cand_p2(k: int) -> float:
                if k < len(turns_after):
                    return turns_after[k]
                ref = turns_after[-1] if turns_after else b
                return ref + REROUTE_SCOPE_STEP_M * (k - len(turns_after) + 1)

            # 3) 主動候選搜尋：取最近轉彎路口 (最多搜尋 REROUTE_SCOPE_LEVELS 階，有結果即終止)
            all_alts: List[Dict[str, Any]] = []
            seen = set()
            total_routes = 0
            total_nonfake = 0
            total_reverted = 0
            best_nonfake: Optional[float] = None
            for k in range(REROUTE_SCOPE_LEVELS):
                p1 = max(evt_start, min(_cand_p1(k), a))
                p2 = min(evt_end, max(_cand_p2(k), b))
                if p2 - p1 < REROUTE_MIN_WINDOW_M:
                    continue
                alts, stats = _reroute_candidates(path, cum, main_segs, main_ivs, vds, p1, p2,
                                                  leg_start, leg_end, main_total_s,
                                                  req.replaced_segments or [])
                total_routes += int(stats.get("osrm_routes") or 0)
                total_nonfake += int(stats.get("nonfake_alts") or 0)
                total_reverted += int(stats.get("reverted_alts") or 0)
                stat_best = stats.get("best_nonfake_saved_min")
                if isinstance(stat_best, (int, float)):
                    best_nonfake = stat_best if best_nonfake is None else max(best_nonfake, stat_best)
                for alt in alts:
                    alt["scope"] = k
                    key = (alt["p1_m"], alt["p2_m"],
                           round(float(alt["distance_km"]), 2),
                           round(float(alt["saved_min"]), 1))
                    if key in seen:
                        continue
                    seen.add(key)
                    all_alts.append(alt)
                if all_alts:
                    break
            if all_alts:
                all_alts.sort(key=lambda a: (-a["saved_min"], a["eta_min"]))
                all_alts = all_alts[:REROUTE_ALT_MAX]
                best_alt = all_alts[0]
                leg_items.append({
                    **base,
                    "state": "CAN_REROUTE",
                    "reason": None,
                    "scope": best_alt.get("scope", 0),
                    "current_eta_min": round(main_total_s / 60.0, 1),
                    "best_saved_min": best_alt["saved_min"],
                    "alternatives": all_alts,
                })
            else:
                if total_routes == 0:
                    reason = "NO_ALTERNATIVE"
                elif total_nonfake == 0:
                    reason = "FAKE_ALTERNATIVE"
                elif total_reverted >= total_nonfake:
                    reason = "REVERTED_ORIGINAL"
                else:
                    reason = "NO_TIME_SAVED"
                leg_items.append({
                    **base,
                    "state": "CANNOT_REROUTE",
                    "reason": reason,
                    "best_nonfake_saved_min": round(best_nonfake, 1) if best_nonfake is not None else None,
                })

        # 比對道路及時間節省，合併減少同 Leg 內重複的改道事件
        deduped = _deduplicate_leg_reroutes(leg_items)
        out_legs.extend(deduped)

    return {"status": "success", "legs": out_legs}


def detect_turn_boundaries(path: List[List[float]], cum: List[float], min_angle_deg: float = 35.0) -> List[float]:
    """從折線幾何中，根據前後向量轉向角 (>35度) 推算路口轉彎點里程。"""
    turns: List[float] = []
    if len(path) < 3 or len(cum) != len(path):
        return turns
    for i in range(1, len(path) - 1):
        d_prev = cum[i] - cum[i - 1]
        d_next = cum[i + 1] - cum[i]
        if d_prev < 3.0 or d_next < 3.0:
            continue
        lat0, lon0 = path[i - 1]
        lat1, lon1 = path[i]
        lat2, lon2 = path[i + 1]
        v1_y = lat1 - lat0
        v1_x = (lon1 - lon0) * math.cos(math.radians(lat1))
        v2_y = lat2 - lat1
        v2_x = (lon2 - lon1) * math.cos(math.radians(lat1))
        heading1 = math.atan2(v1_y, v1_x)
        heading2 = math.atan2(v2_y, v2_x)
        diff = abs(math.degrees(heading2 - heading1))
        if diff > 180.0:
            diff = 360.0 - diff
        if diff >= min_angle_deg:
            if not turns or (cum[i] - turns[-1] >= 25.0):
                turns.append(round(cum[i], 1))
    return turns


class RouteProfilePiece(BaseModel):
    """拼接用路段：幾何 + 自身剖面（里程相對於該段起點）。"""
    path: List[List[float]] = []
    speed_segments: List[List[float]] = []
    road_segments: List[List[Any]] = []
    turn_boundaries: List[float] = []


class RouteReprofileRequest(BaseModel):
    polyline: List[List[float]] = []          # 舊格式：只有幾何（無剖面）
    pieces: List[RouteProfilePiece] = []      # 新格式：原路前段 + 替代路 + 原路後段
    counties: List[str] = []
    leg_boundaries: List[float] = []
    stop_coordinates: List[List[float]] = []


def _splice_pieces(pieces: List[Dict[str, Any]]) -> Dict[str, Any]:
    """依序拼接各段幾何與剖面；段與段的接合點視為轉彎點（截斷偵測器覆蓋）。"""
    path: List[List[float]] = []
    cum: List[float] = [0.0]
    turns: List[float] = []
    roads: List[List[Any]] = []
    speeds: List[List[float]] = []
    for k, pc in enumerate(pieces):
        pts = [p for p in (pc.get("path") or []) if p and len(p) >= 2]
        if not pts:
            continue
        offset = cum[-1] + (haversine_distance_meters(path[-1][0], path[-1][1], pts[0][0], pts[0][1]) if path else 0.0)
        for pt in pts:
            if path:
                prev = path[-1]
                if abs(prev[0] - pt[0]) < 1e-9 and abs(prev[1] - pt[1]) < 1e-9:
                    continue
                cum.append(cum[-1] + haversine_distance_meters(prev[0], prev[1], pt[0], pt[1]))
            path.append([pt[0], pt[1]])
        for a, b, v in (pc.get("speed_segments") or []):
            speeds.append([float(a) + offset, float(b) + offset, float(v)])
        for a, b, name in (pc.get("road_segments") or []):
            roads.append([float(a) + offset, float(b) + offset, name])
        turns.extend(float(t) + offset for t in (pc.get("turn_boundaries") or []))
        if k > 0:
            turns.append(offset)
    total = cum[-1] if cum else 0.0
    return {
        "path": path,
        "cum": cum,
        "turn_boundaries": sorted({round(t, 1) for t in turns if 0.0 < t < total}),
        "road_segments": roads,
        "speed_segments": _normalize_speed_segments(speeds, total),
    }


@app.post("/api/route/reprofile")
def route_reprofile(req: RouteReprofileRequest, force: bool = False):
    """套用改道後，將拼接後的新路線送回後端重新進行完整的路況與偵測器比對：
    1. 拼接原路前段 + 替代路 + 原路後段的幾何與剖面（基準速度、路名、轉彎點）
    2. 依站點順序投影計算各 leg 終點里程 leg_boundaries
    3. 由新剖面重算各 leg 距離與基準時間
    4. 呼叫 compute_vd_overlay 重新投影偵測器，替代路上的 VD 正式標記為 on_route: true
    """
    if req.pieces:
        pieces = [pc.model_dump() if hasattr(pc, "model_dump") else pc.dict() for pc in req.pieces]
        prof = _splice_pieces(pieces)
        path, cum = prof["path"], prof["cum"]
        if len(path) < 2:
            return {"status": "error", "message": "Polyline too short"}
        turns = prof["turn_boundaries"]
    else:
        path = req.polyline or []
        if len(path) < 2:
            return {"status": "error", "message": "Polyline too short"}
        cum = [0.0]
        for i in range(1, len(path)):
            cum.append(cum[-1] + haversine_distance_meters(
                path[i - 1][0], path[i - 1][1], path[i][0], path[i][1]))
        turns = detect_turn_boundaries(path, cum)
        prof = {"road_segments": [], "speed_segments": _normalize_speed_segments([], cum[-1])}

    # 各 leg 終點：依站點順序投影（只往前找），最後一站固定為路線終點
    if req.stop_coordinates and len(req.stop_coordinates) > 1:
        leg_bounds: List[float] = []
        prev = 0.0
        for coord in req.stop_coordinates[1:]:
            _, s = _project_point_to_path(coord[0], coord[1], path, cum, min_s=prev)
            s = max(s, prev)
            leg_bounds.append(round(s, 1))
            prev = s
        leg_bounds[-1] = cum[-1]
    else:
        leg_bounds = req.leg_boundaries if req.leg_boundaries else [cum[-1]]

    profile = {
        "path": path,
        "cum": cum,
        "turn_boundaries": turns,
        "leg_boundaries": leg_bounds,
        "road_segments": prof["road_segments"],
        "speed_segments": prof["speed_segments"],
    }

    legs_out = []
    prev = 0.0
    for i, end in enumerate(leg_bounds):
        legs_out.append({
            "leg_index": i,
            "distance_km": round(max(0.0, end - prev) / 1000.0, 2),
            "base_duration_min": round(base_time_s(profile["speed_segments"], prev, end) / 60.0, 1),
        })
        prev = end

    try:
        vd_points, vd_coverage = compute_vd_overlay(profile, req.counties, force_live=force)
    except Exception as e:
        print(f"[reprofile] compute_vd_overlay 失敗: {e}")
        vd_points, vd_coverage = [], []

    return {
        "status": "success",
        "polyline": path,
        "counties": req.counties,
        "legs": legs_out,
        "vd_points": vd_points,
        "vd_coverage": vd_coverage,
        "route_meta": {
            "cum": cum,
            "turn_boundaries": turns,
            "leg_boundaries": leg_bounds,
            "road_segments": profile["road_segments"],
            "speed_segments": profile["speed_segments"],
        }
    }


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