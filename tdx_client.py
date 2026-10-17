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
_LOCK = threading.Lock()

STATIC_TTL_SEC = 24 * 3600
LIVE_TTL_SEC = 60


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
            time.sleep(20 * (attempt + 1))
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


def live_map(cities: List[str]) -> Dict[str, Dict[str, Any]]:
    """回傳 {VDID: {speed, status, collect_time}}。"""
    result: Dict[str, Dict[str, Any]] = {}
    for city in cities:
        for entry in get_city_vd_live(city):
            vdid = str(entry.get("VDID", ""))
            if not vdid:
                continue
            result[vdid] = {
                "speed": extract_speed(entry),
                "status": entry.get("Status"),
                "collect_time": entry.get("DataCollectTime"),
            }
    return result
