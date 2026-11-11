"""參數合理性檢驗：計時下限 8 km/h、改道省時門檻 0.5 分。

兩個模式：

  collect  定時收集 TDX 偵測器即時資料（每個方向、每個車道的車速、車流量、佔有率）
      python tools/eval_params.py collect --cities Taoyuan,Taipei --every 300 --until "2026-11-12 19:30" \
          --windows "07:00-09:30,11:30-13:00,16:30-19:30"
      （--windows 只在指定時段收集，避開深夜等無效時段，節省 TDX 呼叫次數）
      輸出：eval_output/tdx_snapshots/<城市>_<日期>.jsonl.gz（每次快照一行，可中斷後續跑）

  analyze  分析收集到的資料
      python tools/eval_params.py analyze
      A. 低速讀值：低於 8 km/h 的比例、車流量是否很少、下一筆是否很快恢復
      B. 車速波動：同一方向相隔約 5 分鐘的車速變化，換算成通過 500 m 的時間變化（分）
         ——若一般波動就接近 0.5 分，省時低於 0.5 分的改道很可能只是雜訊
      輸出：eval_output/param_check_summary.json

車速計算方式與系統相同：同一方向（LinkID）所有車道中車速 > 0 者簡單平均。
"""
import argparse
import datetime as dt
import glob
import gzip
import json
import math
import os
import sys
import time

ROOT = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
sys.path.insert(0, ROOT)
import tdx_client  # noqa: E402

OUT_DIR = os.path.join(ROOT, "eval_output")
SNAP_DIR = os.path.join(OUT_DIR, "tdx_snapshots")
SEGMENT_M = 500.0          # 一支偵測器負責的典型長度（前後各 500 m 的單側）
FLOOR_KMH = 8.0            # 系統計時下限


# ---------------------------------------------------------------- collect
def _snapshot(city):
    bearings = {}
    for vd in tdx_client.get_city_vds(city):
        for dl in vd.get("DetectionLinks") or []:
            if dl.get("LinkID"):
                bearings[dl["LinkID"]] = dl.get("Bearing") or dl.get("RoadDirection") or ""
    vds = []
    for e in tdx_client.get_city_vd_live(city, use_cache=False):
        links = []
        for f in e.get("LinkFlows") or []:
            lanes = []
            for ln in f.get("Lanes") or []:
                vol = sum(int(v.get("Volume") or 0) for v in (ln.get("Vehicles") or []))
                lanes.append({"s": ln.get("Speed"), "v": vol, "o": ln.get("Occupancy"), "e": ln.get("ErrorType")})
            links.append({"lid": f.get("LinkID"), "b": bearings.get(f.get("LinkID"), ""), "lanes": lanes})
        vds.append({"id": e.get("VDID"), "t": e.get("DataCollectTime"), "st": e.get("Status"), "links": links})
    return vds


def _parse_windows(spec):
    """'07:00-09:30,16:30-19:30' → [(time, time), ...]；空字串表示全天收集。"""
    out = []
    for part in (spec or "").split(","):
        part = part.strip()
        if part:
            a, b = part.split("-")
            out.append((dt.datetime.strptime(a, "%H:%M").time(), dt.datetime.strptime(b, "%H:%M").time()))
    return out


def _next_window_start(now, windows):
    """目前不在收集時段時，回傳下一個時段的開始時間（節省 TDX 呼叫次數）。"""
    if not windows or any(a <= now.time() < b for a, b in windows):
        return None
    starts = [dt.datetime.combine(now.date(), a) for a, _ in windows]
    future = [s for s in starts if s > now]
    return min(future) if future else dt.datetime.combine(now.date() + dt.timedelta(days=1), min(a for a, _ in windows))


def collect(cities, every, until, windows=""):
    os.makedirs(SNAP_DIR, exist_ok=True)
    end = dt.datetime.strptime(until, "%Y-%m-%d %H:%M")
    wins = _parse_windows(windows)
    log = open(os.path.join(OUT_DIR, "collect.log"), "a", encoding="utf-8")
    if wins:
        log.write(f"{dt.datetime.now():%H:%M:%S} windows {windows}\n")
        log.flush()
    while dt.datetime.now() < end:
        now = dt.datetime.now()
        nxt_start = _next_window_start(now, wins)
        if nxt_start is not None:  # 不在收集時段：睡到下一個時段（或結束時間）
            time.sleep(max(5, (min(nxt_start, end) - now).total_seconds()))
            continue
        for city in cities:
            try:
                time.sleep(5)  # 城市之間稍停，避免觸發 TDX 每秒呼叫上限
                vds = _snapshot(city)
                if not vds:  # 暫時失敗（如 429）：稍候重試一次
                    time.sleep(30)
                    vds = _snapshot(city)
                path = os.path.join(SNAP_DIR, f"{city}_{now:%Y%m%d}.jsonl.gz")
                with gzip.open(path, "at", encoding="utf-8") as f:
                    f.write(json.dumps({"fetched_at": now.isoformat(timespec="seconds"), "city": city, "vds": vds},
                                       ensure_ascii=False) + "\n")
                log.write(f"{now:%H:%M:%S} {city} ok {len(vds)} VDs\n")
            except Exception as exc:  # 網路或 TDX 暫時失敗：記錄後繼續
                log.write(f"{now:%H:%M:%S} {city} ERROR {exc!r}\n")
            log.flush()
        # 對齊到下一個整點區間（例如每 5 分鐘的 :01、:06…，讓 TDX 該期資料已更新）
        nxt = (int(time.time()) // every + 1) * every + 60
        time.sleep(max(5, nxt - time.time()))
    log.write(f"{dt.datetime.now():%H:%M:%S} done\n")
    log.close()


# ---------------------------------------------------------------- analyze
def _readings():
    """回傳 {(city, lid): [(collect_time, speed, volume), ...]}（依時間排序、同一時間去重）。"""
    series = {}
    for path in sorted(glob.glob(os.path.join(SNAP_DIR, "*.jsonl.gz"))):
        with gzip.open(path, "rt", encoding="utf-8") as f:
            for line in f:
                snap = json.loads(line)
                for vd in snap["vds"]:
                    if vd.get("st") not in (None, 0, "0") or not vd.get("t"):
                        continue
                    t = dt.datetime.fromisoformat(vd["t"])
                    for lk in vd["links"]:
                        sp = [ln["s"] for ln in lk["lanes"] if isinstance(ln.get("s"), (int, float)) and ln["s"] > 0]
                        if not sp:
                            continue
                        speed = sum(sp) / len(sp)
                        if speed > 150:
                            continue
                        vol = sum(int(ln.get("v") or 0) for ln in lk["lanes"])
                        series.setdefault((snap["city"], lk["lid"]), {})[t] = (speed, vol)
    return {k: sorted((t, s, v) for t, (s, v) in d.items()) for k, d in series.items()}


def _pct(vals, q):
    if not vals:
        return None
    vals = sorted(vals)
    i = min(len(vals) - 1, max(0, int(math.ceil(q / 100 * len(vals))) - 1))
    return round(vals[i], 2)


def _seg_min(speed):
    return SEGMENT_M / (max(speed, FLOOR_KMH) / 3.6) / 60.0


def analyze():
    series = _readings()
    allr = [(s, v, t.hour) for ser in series.values() for t, s, v in ser]
    n = len(allr)
    if not n:
        print("沒有資料，請先執行 collect")
        return
    low = [(s, v) for s, v, _ in allr if s < FLOOR_KMH]
    norm = [(s, v) for s, v, _ in allr if s >= FLOOR_KMH]

    def vol_dist(rows):
        k = len(rows) or 1
        return {"0-1": round(sum(1 for _, v in rows if v <= 1) / k * 100, 1),
                "2-5": round(sum(1 for _, v in rows if 2 <= v <= 5) / k * 100, 1),
                ">5": round(sum(1 for _, v in rows if v > 5) / k * 100, 1)}

    # A. 低速讀值的下一筆（約 5 分鐘後）
    nxt_low = nxt_rec = nxt_n = 0
    deltas, deltas_cong = [], []
    for ser in series.values():
        for (t1, s1, _), (t2, s2, _) in zip(ser, ser[1:]):
            if (t2 - t1).total_seconds() > 360:
                continue
            if s1 < FLOOR_KMH:
                nxt_n += 1
                nxt_low += s2 < FLOOR_KMH
                nxt_rec += s2 >= 15
            d = abs(_seg_min(s1) - _seg_min(s2))
            deltas.append(d)
            if min(s1, s2) < 30:
                deltas_cong.append(d)

    by_hour = {}
    for s, _, h in allr:
        c = by_hour.setdefault(h, [0, 0])
        c[0] += 1
        c[1] += s < FLOOR_KMH
    summary = {
        "direction_readings": n,
        "links": len(series),
        "A_low_speed": {
            "share_below_8_pct": round(len(low) / n * 100, 2),
            "volume_dist_below_8_pct": vol_dist(low),
            "volume_dist_8_and_above_pct": vol_dist(norm),
            "next_reading_pairs": nxt_n,
            "next_still_below_8_pct": round(nxt_low / nxt_n * 100, 1) if nxt_n else None,
            "next_recovered_15plus_pct": round(nxt_rec / nxt_n * 100, 1) if nxt_n else None,
            "share_below_8_by_hour_pct": {h: round(c[1] / c[0] * 100, 2) for h, c in sorted(by_hour.items())},
            "segment_time_min_at": {f"{v} km/h": round(SEGMENT_M / (v / 3.6) / 60, 2) for v in (1, 2, 5, 8, 15, 30)},
        },
        "B_fluctuation_500m_min": {
            "pairs": len(deltas),
            "all": {f"p{q}": _pct(deltas, q) for q in (50, 75, 90, 95)},
            "share_over_0.5_pct": round(sum(d > 0.5 for d in deltas) / len(deltas) * 100, 1) if deltas else None,
            "congested_pairs(<30km/h)": len(deltas_cong),
            "congested": {f"p{q}": _pct(deltas_cong, q) for q in (50, 75, 90, 95)},
            "congested_share_over_0.5_pct": round(sum(d > 0.5 for d in deltas_cong) / len(deltas_cong) * 100, 1) if deltas_cong else None,
        },
    }
    os.makedirs(OUT_DIR, exist_ok=True)
    with open(os.path.join(OUT_DIR, "param_check_summary.json"), "w", encoding="utf-8") as f:
        json.dump(summary, f, ensure_ascii=False, indent=2)
    print(json.dumps(summary, ensure_ascii=False, indent=2))


def main():
    ap = argparse.ArgumentParser()
    sub = ap.add_subparsers(dest="cmd", required=True)
    c = sub.add_parser("collect")
    c.add_argument("--cities", default="Taoyuan,Taipei")
    c.add_argument("--every", type=int, default=300, help="收集間隔（秒）")
    c.add_argument("--until", required=True, help='結束時間，例如 "2026-11-12 19:30"')
    c.add_argument("--windows", default="",
                   help='只在這些時段收集以節省 TDX 次數，例如 "07:00-09:30,11:30-13:00,16:30-19:30"；省略＝全天')
    sub.add_parser("analyze")
    a = ap.parse_args()
    if a.cmd == "collect":
        collect([x.strip() for x in a.cities.split(",") if x.strip()], a.every, a.until, a.windows)
    else:
        analyze()


if __name__ == "__main__":
    main()
