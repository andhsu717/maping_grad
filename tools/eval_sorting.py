"""排序效益評估：比較三種站點順序的實際道路距離與預估時間。

    A. 原始順序      ：Excel 中的站點順序（不排序）
    B. 直線距離排序  ：以直線距離（Haversine）跑貪婪法 + 2-opt
    C. 道路距離排序  ：以 OSRM 道路距離跑貪婪法 + 2-opt（本系統做法）

三種方案的站點合併、出發點指派、排序演算法與計時規則完全相同，唯一差別是「排序時用的距離」。
評分一律用 OSRM 實際道路路線（compute_route_with_road_speed）：道路距離 km、路名速限預估時間 min。

車次分組與出發點指派比照網頁前端：
  - 車次 = 派車日期 + 派車單號
  - 出發點 = 離該車次站點中心最近的營業所

用法（需先啟動 OSRM）：
    python tools/eval_sorting.py --stops 北區貨運紀錄.xlsx --depots 營業所清單.xlsx --out eval_output
輸出：
    eval_output/sorting_by_trip.csv   每個車次的三種方案結果
    eval_output/sorting_summary.json  彙總
"""
import argparse
import json
import math
import os
import sys

import pandas as pd

sys.path.insert(0, os.path.dirname(os.path.dirname(os.path.abspath(__file__))))
import optimizer_service as svc  # noqa: E402


def _find_col(cols, keywords):
    for c in cols:
        if any(k in str(c).lower() for k in keywords):
            return c
    return None


def _coord_cols(df):
    """比照前端 classifyCoords：依欄名與數值範圍找經緯度欄位。"""
    lat_col = lon_col = None
    for c in df.columns:
        s = pd.to_numeric(df[c], errors="coerce").dropna()
        s = s[s != 0].head(30)
        if s.empty:
            continue
        avg = float(s.mean())
        low = str(c).lower()
        if 118 < avg < 123 and any(k in low for k in ("lon", "lng", "x", "經度")):
            lon_col = c
        elif 21 < avg < 26 and any(k in low for k in ("lat", "y", "緯度")):
            lat_col = c
    return lat_col, lon_col


def load_depots(path):
    df = pd.read_excel(path)
    lat_col, lon_col = _coord_cols(df)
    name_col = _find_col(df.columns, ("名稱", "起點", "倉庫", "站名")) or df.columns[0]
    out = []
    for _, r in df.iterrows():
        try:
            lat, lon = float(r[lat_col]), float(r[lon_col])
        except (TypeError, ValueError):
            continue
        if math.isnan(lat) or math.isnan(lon):
            continue
        out.append({"id": str(r[name_col]), "name": str(r[name_col]), "lat": lat, "lon": lon})
    return out


def load_trips(path):
    df = pd.read_excel(path)
    lat_col, lon_col = _coord_cols(df)
    date_col = _find_col(df.columns, ("日期", "時間", "出貨日", "date"))
    order_col = _find_col(df.columns, ("組別", "派車單號", "車次", "行程")) or _find_col(df.columns, ("單號",))
    stop_col = _find_col(df.columns, ("客單編號", "出貨單號", "站點編號", "包裹"))
    addr_col = _find_col(df.columns, ("提貨人地址", "正規化地址", "地址", "客戶地址"))
    trips = {}
    for idx, r in df.iterrows():
        lat = pd.to_numeric(r[lat_col], errors="coerce")
        lon = pd.to_numeric(r[lon_col], errors="coerce")
        if pd.isna(lat) or pd.isna(lon) or lat == 0 or lon == 0:
            continue
        key = (str(r[date_col]) if date_col else "未分類日期", str(r[order_col]) if order_col else "未分類行程")
        trips.setdefault(key, []).append(svc.DeliveryStop(
            stop_id=str(r[stop_col]) if stop_col else f"STOP_{idx + 1}",
            address=str(r[addr_col]) if addr_col and not pd.isna(r[addr_col]) else "",
            lat=float(lat), lon=float(lon),
        ))
    return trips


def nearest_depot(depots, stops):
    clat = sum(s.lat for s in stops) / len(stops)
    clon = sum(s.lon for s in stops) / len(stops)
    return min(depots, key=lambda d: math.hypot(d["lat"] - clat, d["lon"] - clon))


def score(coords, order):
    """以 OSRM 實際道路路線評分（不回倉庫）。"""
    r = svc.compute_route_with_road_speed([coords[i] for i in order])
    return r["total_distance_km"], r["total_duration_min"], bool(r["profile"]["road_segments"])


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--stops", required=True)
    ap.add_argument("--depots", required=True)
    ap.add_argument("--out", default="eval_output")
    args = ap.parse_args()
    os.makedirs(args.out, exist_ok=True)

    depots = load_depots(args.depots)
    trips = load_trips(args.stops)
    rows = []
    for n, ((date, order_id), stops) in enumerate(sorted(trips.items()), 1):
        clusters = svc.merge_stops_by_proximity_or_address(stops, threshold_meters=30.0)
        depot = nearest_depot(depots, stops)
        coords = [[depot["lon"], depot["lat"]]] + [[c["lon"], c["lat"]] for c in clusters]
        nodes = list(range(len(coords)))
        order_a = nodes
        order_b = svc.open_tsp_2opt(nodes, svc.compute_haversine_matrix(coords), start_idx=0)
        order_c = svc.open_tsp_2opt(nodes, svc.fetch_osrm_table(coords), start_idx=0)
        res = {}
        ok = True
        for tag, order in (("A", order_a), ("B", order_b), ("C", order_c)):
            km, mins, osrm_ok = score(coords, order)
            ok = ok and osrm_ok
            res[tag] = (km, mins)
        rows.append({
            "date": date, "order_id": order_id, "depot": depot["name"],
            "rows": len(stops), "stops_after_merge": len(clusters),
            "A_km": res["A"][0], "A_min": res["A"][1],
            "B_km": res["B"][0], "B_min": res["B"][1],
            "C_km": res["C"][0], "C_min": res["C"][1],
            "same_order_BC": order_b == order_c, "osrm_ok": ok,
        })
        if n % 20 == 0:
            print(f"  {n}/{len(trips)} trips")

    df = pd.DataFrame(rows)
    df.to_csv(os.path.join(args.out, "sorting_by_trip.csv"), index=False, encoding="utf-8-sig")
    ev = df[df["osrm_ok"] & (df["stops_after_merge"] >= 2)]

    def pct(a, b):
        return round((a - b) / a * 100, 1) if a else 0.0

    summary = {
        "trips_total": int(len(df)),
        "trips_evaluated": int(len(ev)),
        "delivery_rows": int(df["rows"].sum()),
        "stops_after_merge": int(df["stops_after_merge"].sum()),
        "merge_reduction_pct": pct(df["rows"].sum(), df["stops_after_merge"].sum()),
        "total_km": {k: round(float(ev[f"{k}_km"].sum()), 1) for k in "ABC"},
        "total_min": {k: round(float(ev[f"{k}_min"].sum()), 1) for k in "ABC"},
        "C_vs_A_km_pct": pct(ev["A_km"].sum(), ev["C_km"].sum()),
        "C_vs_B_km_pct": pct(ev["B_km"].sum(), ev["C_km"].sum()),
        "C_vs_A_min_pct": pct(ev["A_min"].sum(), ev["C_min"].sum()),
        "C_vs_B_min_pct": pct(ev["B_min"].sum(), ev["C_min"].sum()),
        "trips_C_better_than_B": int((ev["C_km"] < ev["B_km"] - 0.05).sum()),
        "trips_C_equal_B": int((abs(ev["C_km"] - ev["B_km"]) <= 0.05).sum()),
        "trips_C_worse_than_B": int((ev["C_km"] > ev["B_km"] + 0.05).sum()),
        "per_trip_C_vs_B_km_pct": {
            "median": round(float(((ev["B_km"] - ev["C_km"]) / ev["B_km"] * 100).median()), 1),
            "max": round(float(((ev["B_km"] - ev["C_km"]) / ev["B_km"] * 100).max()), 1),
        },
    }
    with open(os.path.join(args.out, "sorting_summary.json"), "w", encoding="utf-8") as f:
        json.dump(summary, f, ensure_ascii=False, indent=2)
    print(json.dumps(summary, ensure_ascii=False, indent=2))


if __name__ == "__main__":
    main()
