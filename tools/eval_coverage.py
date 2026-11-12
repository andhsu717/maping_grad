"""各縣市車流偵測器（VD）涵蓋統計。

對系統支援的每個縣市，取得 TDX 設備清單與即時資料，統計：
  - 設備數：靜態清單中的 VD 數量
  - 有即時資料：即時資料中設備狀態正常、且至少一個車道車速 > 0 的 VD 數
  - 即時回報率：有即時資料 ÷ 設備數

呼叫額度：TDX 一般會員每分鐘 5 次，由 tdx_client 自動排隊等待（14 個縣市約需 5～10 分鐘）。

用法：
    python tools/eval_coverage.py
輸出：eval_output/county_coverage.json
"""
import datetime as dt
import json
import os
import sys

ROOT = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
sys.path.insert(0, ROOT)
import tdx_client  # noqa: E402

NAMES = {}
for name, code in tdx_client.CITY_CODE_MAP.items():
    if name.endswith(("市", "縣")) and code not in NAMES:
        NAMES[code] = name.replace("台", "臺")


def main():
    out = []
    started = dt.datetime.now()
    for code, name in NAMES.items():
        vds = tdx_client.get_city_vds(code)
        live = tdx_client.get_city_vd_live(code, use_cache=False)
        reporting = 0
        for e in live:
            if e.get("Status") not in (None, 0, "0"):
                continue
            if any(isinstance(ln.get("Speed"), (int, float)) and ln["Speed"] > 0
                   for f in (e.get("LinkFlows") or []) for ln in (f.get("Lanes") or [])):
                reporting += 1
        row = {"code": code, "name": name, "vds": len(vds), "live_entries": len(live), "reporting": reporting,
               "reporting_pct": round(reporting / len(vds) * 100, 1) if vds else 0.0}
        out.append(row)
        print(f"{name:5s} 設備 {row['vds']:4d}  即時筆數 {row['live_entries']:4d}  有車速 {reporting:4d}  回報率 {row['reporting_pct']:5.1f}%",
              flush=True)
    out.sort(key=lambda r: -r["vds"])
    result = {"collected_at": started.isoformat(timespec="minutes"), "counties": out}
    os.makedirs(os.path.join(ROOT, "eval_output"), exist_ok=True)
    with open(os.path.join(ROOT, "eval_output", "county_coverage.json"), "w", encoding="utf-8") as f:
        json.dump(result, f, ensure_ascii=False, indent=2)
    print(f"完成，耗時 {(dt.datetime.now() - started).seconds} 秒")


if __name__ == "__main__":
    main()
