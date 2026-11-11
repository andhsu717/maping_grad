"""
TDX 車輛偵測器 (VD) 資料用戶端
- 憑證來源：key.env / .env（支援 `ID:`/`SERECT:` 與標準 `CLIENT_ID=`/`CLIENT_SECRET=`）
- 靜態端點：GET /v2/Road/Traffic/VD/City/{City}        （設備清單，含經緯度）
- 即時端點：GET /v2/Road/Traffic/Live/VD/City/{City}   （即時速度/流量）
- token 與回應皆快取，遇 429 自動退避重試
"""
import os
import time
import threading
from typing import Any, Dict, List, Optional, Tuple

import requests

BASE_DIR = os.path.dirname(os.path.abspath(__file__))
TDX_TOKEN_URL = "https://tdx.transportdata.tw/auth/realms/TDXConnect/protocol/openid-connect/token"
TDX_BASE = "https://tdx.transportdata.tw/api/basic/v2/Road/Traffic"

HEADERS_BASE = {
    "User-Agent": "SmartLogisticsApp/2.0 (TDX-Client)",
    "Accept": "application/json",
}

# 地址關鍵字 -> TDX VD City 服務支援的縣市代碼
CITY_CODE_MAP: Dict[str, str] = {
    "臺北市": "Taipei", "台北市": "Taipei",
    "桃園市": "Taoyuan", "桃園縣": "Taoyuan", "中壢": "Taoyuan", "平鎮": "Taoyuan",
    "臺中市": "Taichung", "台中市": "Taichung",
    "臺南市": "Tainan", "台南市": "Tainan",
    "高雄市": "Kaohsiung",
    "基隆市": "Keelung",
    "新竹縣": "HsinchuCounty",
    "南投縣": "NantouCounty",
    "雲林縣": "YunlinCounty",
    "嘉義縣": "ChiayiCounty",
    "屏東縣": "PingtungCounty",
    "臺東縣": "TaitungCounty", "台東縣": "TaitungCounty",
    "花蓮縣": "HualienCounty",
    "金門縣": "KinmenCounty",
}

_TOKEN_CACHE: Dict[str, Any] = {"token": None, "exp": 0.0}
_STATIC_CACHE: Dict[str, Dict[str, Any]] = {}
_LIVE_CACHE: Dict[str, Dict[str, Any]] = {}
_LEVEL_CACHE: Dict[str, Dict[str, Any]] = {}
_SECTION_CACHE: Dict[str, Dict[str, Any]] = {}
_LOCK = threading.Lock()

STATIC_TTL_SEC = 24 * 3600
LIVE_TTL_SEC = 60
CONGESTION_TTL_SEC = 6 * 3600

# 公路局 fallback 門檻（無官方逐路段定義時使用，單位 km/h）
FALLBACK_LEVEL_SPEED = [(40.0, 1), (20.0, 3), (0.0, 5)]
# 各等級代表速度（無官方門檻時；模擬壅塞用）
FALLBACK_REP_SPEED = {1: 45.0, 2: 37.0, 3: 30.0, 4: 23.0, 5: 10.0}

# 車速上限（km/h）：大於此值視為偵測器異常；車速 0 的車道視為無資料、不納入平均
SPEED_VALID_MAX = 150.0

# TDX 方位字串 → 角度（0=北，順時針）
BEARING_DEG = {"N": 0.0, "NE": 45.0, "E": 90.0, "SE": 135.0,
               "S": 180.0, "SW": 225.0, "W": 270.0, "NW": 315.0}
SAME_DIRECTION_MAX_DEG = 67.5   # 與路線前進方向夾角在此內視為同方向
NEAREST_DIRECTION_MAX_DEG = 90.0  # 無同方向時，取最接近且小於此角度者


def _load_credentials() -> Tuple[Optional[str], Optional[str]]:
    cid = sec = None
    for fname in ("key.env", ".env"):
        path = os.path.join(BASE_DIR, fname)
        if not os.path.exists(path):
            continue
        try:
            with open(path, "r", encoding="utf-8-sig") as f:
                for raw in f:
                    line = raw.strip()
                    if not line or line.startswith("#"):
                        continue
                    sep = "=" if "=" in line else (":" if ":" in line else None)
                    if not sep:
                        continue
                    key, val = line.split(sep, 1)
                    key = key.strip().upper().replace(" ", "")
                    val = val.strip().strip('"').strip("'")
                    if key in ("ID", "CLIENT_ID", "CLIENTID"):
                        cid = val
                    elif key in ("SERECT", "SECRET", "CLIENT_SECRET", "CLIENTSECRET"):
                        sec = val
        except OSError:
            continue
        if cid and sec:
            return cid, sec
    cid = cid or os.environ.get("TDX_CLIENT_ID")
    sec = sec or os.environ.get("TDX_CLIENT_SECRET")
    return cid, sec


def get_token(force: bool = False) -> Optional[str]:
    with _LOCK:
        now = time.time()
        if not force and _TOKEN_CACHE["token"] and now < _TOKEN_CACHE["exp"] - 120:
            return _TOKEN_CACHE["token"]
        cid, sec = _load_credentials()
        if not cid or not sec:
            return None
        try:
            resp = requests.post(
                TDX_TOKEN_URL,
                data={"grant_type": "client_credentials", "client_id": cid, "client_secret": sec},
                timeout=30,
            )
            if resp.status_code != 200:
                return None
            data = resp.json()
            _TOKEN_CACHE["token"] = data.get("access_token")
            _TOKEN_CACHE["exp"] = now + int(data.get("expires_in", 86400))
            return _TOKEN_CACHE["token"]
        except requests.exceptions.RequestException:
            return None


def _get(path: str, params: Optional[Dict[str, Any]] = None) -> Optional[Any]:
    params = dict(params or {})
    params.setdefault("$format", "JSON")
    url = f"{TDX_BASE}{path}"
    for attempt in range(4):
        token = get_token(force=(attempt > 0 and attempt < 2))
        headers = dict(HEADERS_BASE)
        if token:
            headers["Authorization"] = f"Bearer {token}"
        try:
            resp = requests.get(url, params=params, headers=headers, timeout=60)
        except requests.exceptions.RequestException:
            time.sleep(2)
            continue
        if resp.status_code == 200:
            try:
                return resp.json()
            except ValueError:
                return None
        if resp.status_code in (401, 403):
            _TOKEN_CACHE["token"] = None
            time.sleep(1)
            continue
        if resp.status_code == 429:
            time.sleep(3 * (attempt + 1))
            continue
        return None
    return None


def get_city_vds(city: str) -> List[Dict[str, Any]]:
    """靜態 VD 設備清單（含 PositionLat/PositionLon），長快取。"""
    now = time.time()
    cached = _STATIC_CACHE.get(city)
    if cached and now < cached["ts"] + STATIC_TTL_SEC:
        return cached["data"]
    data = _get(f"/VD/City/{city}")
    vds = (data or {}).get("VDs", []) if isinstance(data, dict) else []
    if vds:
        _STATIC_CACHE[city] = {"ts": now, "data": vds}
    return vds


def get_city_vd_live(city: str, use_cache: bool = True) -> List[Dict[str, Any]]:
    """即時 VD 車流（速度/流量），短快取。"""
    now = time.time()
    cached = _LIVE_CACHE.get(city)
    if use_cache and cached and now < cached["ts"] + LIVE_TTL_SEC:
        return cached["data"]
    data = _get(f"/Live/VD/City/{city}")
    vds = (data or {}).get("VDLives", []) if isinstance(data, dict) else []
    _LIVE_CACHE[city] = {"ts": now, "data": vds}
    return vds


def detect_counties(addresses: List[str]) -> List[str]:
    """由地址字串關鍵字判定縣市（僅回傳 VD 服務有支援的縣市代碼）。"""
    codes: List[str] = []
    for addr in addresses or []:
        text = str(addr or "")
        for keyword, code in CITY_CODE_MAP.items():
            if keyword in text and code not in codes:
                codes.append(code)
                break
    return codes


def _flow_lane_speeds(flow: Dict[str, Any]) -> List[float]:
    """單一方向 (LinkFlow) 各車道車速；車速 0 的車道（無車或診斷值）不納入。"""
    out: List[float] = []
    for lane in flow.get("Lanes", []) or []:
        spd = lane.get("Speed")
        if isinstance(spd, (int, float)) and spd > 0:
            out.append(float(spd))
    return out


def is_speed_valid(speed: Optional[float]) -> bool:
    """車速是否落在合理範圍（> 0 且 ≤ 150 km/h）。超出視為偵測器異常。"""
    if not isinstance(speed, (int, float)):
        return False
    return 0.0 < float(speed) <= SPEED_VALID_MAX


def static_links(vd: Dict[str, Any]) -> Dict[str, Dict[str, Any]]:
    """由靜態設備清單建立 {LinkID: {bearing, lanes, levels}}（尚無即時車道資料）。"""
    out: Dict[str, Dict[str, Any]] = {}
    for dl in (vd.get("DetectionLinks") or []):
        lid = dl.get("LinkID")
        if lid:
            out[lid] = {
                "bearing": str(dl.get("Bearing") or dl.get("RoadDirection") or "").upper(),
                "lanes": [],
                "levels": None,
            }
    return out


def empty_entry(vd: Dict[str, Any]) -> Dict[str, Any]:
    """無即時資料的偵測器：保留方向資訊，讀值為「無資料」。"""
    base = {"status": None, "collect_time": None, "links": static_links(vd), "vd_levels": None}
    base.update(summary_reading(base))
    return base


def _angle_diff(a: float, b: float) -> float:
    d = abs(a - b) % 360.0
    return 360.0 - d if d > 180.0 else d


def links_for_heading(links: Dict[str, Dict[str, Any]], heading: Optional[float]) -> Optional[List[str]]:
    """依路線前進方位角挑出同方向的 LinkID。

    - 回傳 list：要納入計算的 LinkID（同方向的視為同一條道路）
    - 回傳 None：偵測器只量測對向，不屬於此行進方向
    - 無方位資訊或無法判斷時，回傳全部 LinkID（沿用舊行為）
    """
    ids = list((links or {}).keys())
    if not ids or heading is None:
        return ids
    diffs: Dict[str, float] = {}
    for lid in ids:
        deg = BEARING_DEG.get(str((links[lid] or {}).get("bearing") or "").upper())
        if deg is not None:
            diffs[lid] = _angle_diff(deg, float(heading))
    if not diffs:
        return ids
    same = [lid for lid, d in diffs.items() if d <= SAME_DIRECTION_MAX_DEG]
    if same:
        return same
    best = min(diffs.values())
    if best < NEAREST_DIRECTION_MAX_DEG:
        return [lid for lid, d in diffs.items() if d == best]
    return None


def rep_speed_for_level(levels: Optional[List[Dict[str, Any]]], level: int) -> float:
    """壅塞等級的代表速度：官方門檻區間中點；無門檻時用預設代表速度。"""
    for lv in (levels or []):
        try:
            if int(lv.get("Level", -1)) != int(level):
                continue
        except (TypeError, ValueError):
            continue
        lo = lv.get("LowValue")
        lo = float(lo) if isinstance(lo, (int, float)) else 0.0
        top = lv.get("TopValue")
        top = float(top) if isinstance(top, (int, float)) else lo + 12.0
        return round((lo + top) / 2.0)
    return FALLBACK_REP_SPEED.get(int(level), 0.0)


def direction_reading(entry: Optional[Dict[str, Any]], link_ids: Optional[List[str]] = None,
                      sim_level: Optional[int] = None) -> Dict[str, Any]:
    """偵測器在指定方向 (link_ids) 的讀值：同方向所有車道（車速 > 0）簡單平均。

    link_ids 為空時使用全部方向。sim_level 為模擬壅塞等級（即時優先，模擬只能加重）。
    回傳 {speed, level, level_name, color, levels, speed_valid, anomaly_reason, manual}。
    """
    entry = entry or {}
    links = entry.get("links") or {}
    ids = [lid for lid in (link_ids if link_ids else list(links.keys())) if lid in links]
    levels = None
    for lid in ids:
        if links[lid].get("levels"):
            levels = links[lid]["levels"]
            break
    if levels is None:
        levels = entry.get("vd_levels")

    speed: Optional[float] = None
    level: Optional[int] = None
    anomaly: Optional[str] = None
    status = entry.get("status")
    if not entry:
        anomaly = "無資料"
    elif status not in (None, 0, "0"):
        anomaly = "設備異常"
    else:
        lanes = [s for lid in ids for s in (links[lid].get("lanes") or [])]
        if not lanes:
            anomaly = "無資料"
        else:
            speed = round(sum(lanes) / len(lanes), 1)
            if not is_speed_valid(speed):
                anomaly = "車速異常"
                speed = None
            else:
                level = classify_congestion(speed, levels)

    manual = False
    if sim_level:
        sim = int(sim_level)
        if level is None or sim > level:
            level = sim
            speed = rep_speed_for_level(levels, sim)
            manual = True
            anomaly = None
    return {
        "speed": speed,
        "level": level,
        "level_name": level_name(levels, level),
        "color": level_color(level),
        "levels": levels,
        "speed_valid": speed is not None and speed > 0,
        "anomaly_reason": anomaly,
        "manual": manual,
    }


def summary_reading(entry: Dict[str, Any]) -> Dict[str, Any]:
    """不分路線方向時的代表讀值：取各方向中較慢（等級較嚴重）者。"""
    best: Optional[Dict[str, Any]] = None
    for lid in (entry.get("links") or {}):
        r = direction_reading(entry, [lid])
        if not r["speed_valid"]:
            continue
        if best is None or ((r["level"] or 0), -(r["speed"] or 0)) > ((best["level"] or 0), -(best["speed"] or 0)):
            best = r
    return best if best is not None else direction_reading(entry, None)


def get_congestion_levels(city: str) -> Dict[str, List[Dict[str, Any]]]:
    """官方路況壅塞水準定義：{CongestionLevelID: [Level 定義...]}（長快取）。"""
    now = time.time()
    cached = _LEVEL_CACHE.get(city)
    if cached and now < cached["ts"] + CONGESTION_TTL_SEC:
        return cached["data"]
    data = _get(f"/CongestionLevel/City/{city}")
    out: Dict[str, List[Dict[str, Any]]] = {}
    groups = data.get("CongestionLevels", []) if isinstance(data, dict) else []
    for item in groups:
        clid = item.get("CongestionLevelID")
        if clid:
            out[clid] = item.get("Levels") or []
    _LEVEL_CACHE[city] = {"ts": now, "data": out}
    return out


def get_city_sections(city: str, use_cache: bool = True) -> List[Dict[str, Any]]:
    """即時路段（含 LinkIDs 與 CongestionLevelID），短快取。"""
    now = time.time()
    cached = _SECTION_CACHE.get(city)
    if use_cache and cached and now < cached["ts"] + LIVE_TTL_SEC:
        return cached["data"]
    data = _get(f"/Live/City/{city}")
    secs = data.get("LiveTraffics", []) if isinstance(data, dict) else []
    _SECTION_CACHE[city] = {"ts": now, "data": secs}
    return secs


def get_link_to_clid(city: str, force_live: bool = False) -> Dict[str, Any]:
    """{LinkID: CongestionLevelID}（由即時路段建立，補足無法直接命中的 VD）。"""
    mapping: Dict[str, Any] = {}
    for section in get_city_sections(city, use_cache=not force_live):
        clid = section.get("CongestionLevelID")
        for link in (section.get("LinkIDs") or []):
            lid = link.get("LinkID") if isinstance(link, dict) else link
            if lid:
                mapping[lid] = clid
    return mapping


def _vd_links(vd: Dict[str, Any]) -> List[str]:
    out: List[str] = []
    for dl in (vd.get("DetectionLinks") or []):
        lid = dl.get("LinkID")
        if lid:
            out.append(lid)
    return out


def _resolve_levels(vd: Dict[str, Any], by_clid: Dict[str, List[Dict[str, Any]]],
                    link2clid: Optional[Dict[str, Any]]) -> Optional[List[Dict[str, Any]]]:
    clid = None
    for lid in _vd_links(vd):
        if lid in by_clid:
            clid = lid
            break
    if clid is None and link2clid:
        for lid in _vd_links(vd):
            if lid in link2clid:
                clid = link2clid[lid]
                break
    return by_clid.get(clid) if clid else None


def classify_congestion(speed: Optional[float], levels: Optional[List[Dict[str, Any]]]) -> Optional[int]:
    """依官方逐路段門檻回傳壅塞等級 1~5；無門檻時用公路局 40/20 fallback。無速度回 None。"""
    if speed is None or speed <= 0:
        return None
    if levels:
        for lv in levels:
            lo = lv.get("LowValue")
            lo = float(lo) if isinstance(lo, (int, float)) else 0.0
            top = lv.get("TopValue")
            if speed > lo and (top is None or speed <= float(top)):
                lv_num = lv.get("Level")
                if isinstance(lv_num, (int, float)):
                    return int(lv_num)
        return None
    for threshold, level in FALLBACK_LEVEL_SPEED:
        if speed > threshold:
            return level
    return None


def level_color(level: Optional[int]) -> str:
    """A/B -> 綠、C/D -> 橘、E -> 紅、未知 -> 灰。"""
    if level in (1, 2):
        return "GREEN"
    if level in (3, 4):
        return "ORANGE"
    if level == 5:
        return "RED"
    return "GRAY"


def level_name(levels: Optional[List[Dict[str, Any]]], level: Optional[int]) -> str:
    if levels and level is not None:
        for lv in levels:
            try:
                if int(lv.get("Level", -1)) == level:
                    return str(lv.get("LevelName") or "")
            except (TypeError, ValueError):
                continue
    return ""


def usable_levels(levels: Optional[List[Dict[str, Any]]]) -> Optional[List[Dict[str, Any]]]:
    """過濾無效的官方門檻：所有 LowValue／TopValue 皆為 0 或空值的定義（實測桃園約 3 成）
    會讓任何車速都落在 A 級，視為「沒有官方門檻」，改用 40／20 預設分界。"""
    if not levels:
        return None
    for lv in levels:
        for k in ("LowValue", "TopValue"):
            v = lv.get(k)
            if isinstance(v, (int, float)) and v > 0:
                return levels
    return None


def _link_levels(lid: str, by_clid: Dict[str, List[Dict[str, Any]]],
                 link2clid: Optional[Dict[str, Any]]) -> Optional[List[Dict[str, Any]]]:
    if lid in by_clid:
        return usable_levels(by_clid[lid])
    if link2clid and lid in link2clid:
        return usable_levels(by_clid.get(link2clid[lid]))
    return None


def live_map(cities: List[str], force_live: bool = False) -> Dict[str, Dict[str, Any]]:
    """回傳 {VDID: {status, collect_time, links, vd_levels, + 代表讀值}}。

    links = {LinkID: {bearing, lanes:[各車道車速>0], levels}}：依方向分開，供路線依行進方向取值。
    代表讀值（speed/level/color/...）為「各方向中較慢者」，僅供不在路線上的偵測器顯示。
    force_live=True 時跳過即時資料快取（速度/路段），每次皆重新向 TDX 取號誌；
    靜態設備清單與壅塞門檻定義仍走長快取。
    """
    result: Dict[str, Dict[str, Any]] = {}
    for city in cities:
        by_clid = get_congestion_levels(city)
        link2clid: Optional[Dict[str, Any]] = None
        static_by_vdid: Dict[str, Dict[str, Any]] = {}
        for vd in get_city_vds(city):
            vdid = str(vd.get("VDID", ""))
            if not vdid:
                continue
            if link2clid is None and not any(lid in by_clid for lid in _vd_links(vd)):
                link2clid = get_link_to_clid(city, force_live=force_live)
            static_by_vdid[vdid] = vd
        for entry in get_city_vd_live(city, use_cache=not force_live):
            vdid = str(entry.get("VDID", ""))
            if not vdid:
                continue
            vd = static_by_vdid.get(vdid) or {}
            vd_levels = usable_levels(_resolve_levels(vd, by_clid, link2clid)) if vd else None
            links = static_links(vd)
            for flow in (entry.get("LinkFlows") or []):
                lid = flow.get("LinkID")
                if not lid:
                    continue
                if lid not in links:
                    links[lid] = {"bearing": "", "lanes": [], "levels": None}
                links[lid]["lanes"] = _flow_lane_speeds(flow)
            for lid, info in links.items():
                info["levels"] = _link_levels(lid, by_clid, link2clid) or vd_levels
            base = {
                "status": entry.get("Status"),
                "collect_time": entry.get("DataCollectTime"),
                "links": links,
                "vd_levels": vd_levels,
            }
            base.update(summary_reading(base))
            result[vdid] = base
    return result
