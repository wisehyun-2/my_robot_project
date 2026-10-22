"""Criterion 20 (no regression): compare the stage-1 and stage-2 acceptance summaries.

    python -m wbr.controllers.mpc_host.regression [--stage1 DIR] [--stage2 DIR]

Reads <stage1>/plan4_<rate>Hz.json (host ramp) and <stage2>/plan4_mpc_<rate>Hz.json (host mpc) for 1000 and 200 Hz.
An item regresses when it passed in stage 1 and fails in stage 2, or when its margin (tightest metric, measured /
limit or limit / measured) is more than 10 % worse: margin_2 > 1.1 margin_1.  The per-metric comparison is reported
as well (stricter than the criterion's wording).  Writes results/plan4_mpc/regression.md.
"""
from __future__ import annotations

import argparse
import json
import math
from pathlib import Path

from wbr import RESULTS_DIR


def _load(path: Path):
    return json.loads(path.read_text(encoding="utf-8")) if path.exists() else None


def _tight(item: dict):
    w = item.get("worst") or {}
    if not w:
        return None, None
    metric, info = max(w.items(), key=lambda kv: kv[1]["margin"] if kv[1]["margin"] is not None else 0)
    return metric, info


def _fmt(v, nd=3):
    if v is None:
        return "-"
    if isinstance(v, float):
        if math.isinf(v):
            return "inf"
        return f"{v:.{nd}f}"
    return str(v)


def compare(s1: dict, s2: dict, ids=None) -> tuple:
    items1 = {it["id"]: it for it in s1["items"]}
    items2 = {it["id"]: it for it in s2["items"]}
    rows, metric_rows = [], []
    for iid in sorted(set(items1) & set(items2)):
        if ids and iid not in ids:
            continue
        a, b = items1[iid], items2[iid]
        ma, ia = _tight(a)
        mb, ib = _tight(b)
        m1 = ia["margin"] if ia else None
        m2 = ib["margin"] if ib else None
        regress = (a["passed"] and not b["passed"]) or (m1 is not None and m2 is not None and math.isfinite(m1) and m2 > 1.1 * m1 + 1e-12)
        rows.append(dict(id=iid, name=a["name"], pass1=a["passed"], pass2=b["passed"], metric1=ma, margin1=m1, metric2=mb, margin2=m2,
                         regress=bool(regress)))
        for metric in sorted(set(a["worst"]) & set(b["worst"])):
            x, y = a["worst"][metric], b["worst"][metric]
            if x["margin"] is None or y["margin"] is None:
                continue
            worse = y["margin"] > 1.1 * x["margin"] + 1e-12 and y["margin"] > 0.05      # ignore metrics far from their limit
            metric_rows.append(dict(id=iid, metric=metric, v1=x["value"], v2=y["value"], m1=x["margin"], m2=y["margin"],
                                    limit=x["limit"], op=x["op"], worse=bool(worse)))
    return rows, metric_rows


def main(argv=None):
    ap = argparse.ArgumentParser()
    ap.add_argument("--stage1", default=str(RESULTS_DIR / "acceptance"))
    ap.add_argument("--stage2", default=str(RESULTS_DIR / "acceptance"))
    ap.add_argument("--out", default=str(RESULTS_DIR / "plan4_mpc" / "regression.md"))
    args = ap.parse_args(argv)
    lines = ["# 2단계 회귀 비교(기준 20): 1단계 램프 호스트 vs MPC 호스트", "",
             f"1단계: `{args.stage1}`, 2단계: `{args.stage2}`. 여유율 = 측정/기준(≤) 또는 기준/측정(≥), 작을수록 여유. "
             "회귀 = 1단계 통과→2단계 실패, 또는 가장 빠듯한 지표의 여유율이 1.1배 넘게 나빠짐.", ""]
    for rate in ("1000", "200"):
        s1 = _load(Path(args.stage1) / f"plan4_{rate}Hz.json")
        s2 = _load(Path(args.stage2) / f"plan4_mpc_{rate}Hz.json")
        if not s1 or not s2:
            lines += [f"## {rate} Hz", "", "요약 파일 없음", ""]
            continue
        rows, mrows = compare(s1, s2)
        lines += [f"## {rate} Hz — 항목별(가장 빠듯한 지표)", "",
                  "| # | 항목 | 1단계 | 2단계 | 1단계 지표(여유율) | 2단계 지표(여유율) | 회귀 |", "|---|---|---|---|---|---|---|"]
        for r in rows:
            lines.append(f"| {r['id']} | {r['name']} | {'통과' if r['pass1'] else '실패'} | {'통과' if r['pass2'] else '실패'} | "
                         f"{r['metric1']} ({_fmt(r['margin1'], 2)}) | {r['metric2']} ({_fmt(r['margin2'], 2)}) | {'**예**' if r['regress'] else '아니오'} |")
        worse = [m for m in mrows if m["worse"]]
        lines += ["", f"### {rate} Hz — 지표별로 10 % 넘게 나빠진 것(여유율 0.05 미만 지표 제외)", ""]
        if worse:
            lines += ["| # | 지표 | 1단계 값 | 2단계 값 | 기준 | 여유율 1→2 |", "|---|---|---|---|---|---|"]
            for m in worse:
                lines.append(f"| {m['id']} | {m['metric']} | {_fmt(m['v1'])} | {_fmt(m['v2'])} | {m['op']} {_fmt(m['limit'])} | "
                             f"{_fmt(m['m1'], 2)} → {_fmt(m['m2'], 2)} |")
        else:
            lines.append("없음")
        lines.append("")
        print(f"{rate} Hz: items regressed: {[r['id'] for r in rows if r['regress']]}; metrics worse >10 %: "
              f"{[(m['id'], m['metric']) for m in worse]}")
    Path(args.out).write_text("\n".join(lines) + "\n", encoding="utf-8")
    print("wrote", args.out)


if __name__ == "__main__":
    main()
