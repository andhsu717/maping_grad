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

# 車速合理範圍（km/h）：超出視為偵測器異常，歸類為綠燈（順暢）
SPEED_VALID_MIN = 0.5
SPEED_VALID_MAX = 150.0


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


def extract_speed(vd_live: Dict[str, Any]) -> float:
    """由一筆 VDLive 取代表速度：回報中所有車道（Speed>0）的平均；無資料回 0。"""
    speeds: List[float] = []
    for flow in vd_live.get("LinkFlows", []) or []:
        for lane in flow.get("Lanes", []) or []:
            spd = lane.get("Speed")
            if isinstance(spd, (int, float)) and spd > 0:
                speeds.append(float(spd))
    if not speeds:
        return 0.0
    return round(sum(speeds) / len(speeds), 1)


def is_speed_valid(speed: Optional[float]) -> bool:
    """車速是否落在合理範圍（0.5~150 km/h）。超出視為偵測器異常。"""
    if not isinstance(speed, (int, float)):
        return False
    return SPEED_VALID_MIN <= float(speed) <= SPEED_VALID_MAX


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


def live_map(cities: List[str], force_live: bool = False) -> Dict[str, Dict[str, Any]]:
    """回傳 {VDID: {speed, status, collect_time, speed_valid, level, level_name, color, levels}}。

    force_live=True 時跳過即時資料快取（速度/路段），每次皆重新向 TDX 取號誌；
    靜態設備清單與壅塞門檻定義仍走長快取。
    車速不合理（<0.5 或 >150 km/h）視為偵測器異常 → 歸類為 GREEN（level 1）。
    """
    result: Dict[str, Dict[str, Any]] = {}
    for city in cities:
        by_clid = get_congestion_levels(city)
        link2clid: Optional[Dict[str, Any]] = None
        levels_by_vdid: Dict[str, Optional[List[Dict[str, Any]]]] = {}
        for vd in get_city_vds(city):
            vdid = str(vd.get("VDID", ""))
            if not vdid:
                continue
            if link2clid is None and not any(lid in by_clid for lid in _vd_links(vd)):
                link2clid = get_link_to_clid(city, force_live=force_live)
            levels_by_vdid[vdid] = _resolve_levels(vd, by_clid, link2clid)
        for entry in get_city_vd_live(city, use_cache=not force_live):
            vdid = str(entry.get("VDID", ""))
            if not vdid:
                continue
            speed = extract_speed(entry)
            status = entry.get("Status")
            levels = levels_by_vdid.get(vdid)
            if speed is None or status not in (None, 0):
                color, level, speed_valid, levels = "GRAY", None, False, None
            elif not is_speed_valid(speed):
                color, level, speed_valid = "GREEN", 1, False
            else:
                level = classify_congestion(speed, levels)
                color = level_color(level)
                speed_valid = True
            result[vdid] = {
                "speed": speed,
                "status": status,
                "collect_time": entry.get("DataCollectTime"),
                "speed_valid": speed_valid,
                "level": level,
                "level_name": level_name(levels, level),
                "color": color,
                "levels": levels,
            }
    return result
