"""Run the staged tests in order and write TEST/results/stage_report.md (+ .json).

    python TEST/run_stages.py                 # stages 1 -> 4, stops at the first stage that fails
    python TEST/run_stages.py --stage 1 2     # only these stages
    python TEST/run_stages.py --keep-going    # run every stage even after a failure
    set WBR_TEST_SEEDS=0,1,2  (PowerShell: $env:WBR_TEST_SEEDS = "0,1,2")   # more noise seeds

A stage builds on the one before it, so a failure stops the run by default: fix that stage first.
"""
from __future__ import annotations

import argparse
import json
import sys
import time
import unittest
from pathlib import Path

HERE = Path(__file__).resolve().parent
sys.path.insert(0, str(HERE))

import stage_common as SC  # noqa: E402

MODULES = {1: "test_stage1_lqr", 2: "test_stage2_outer_loops", 3: "test_stage3_int_position", 4: "test_stage4_int_tilt"}
TITLES = {1: "순수 LQR", 2: "바깥 루프(높이·요·롤·벌어짐)", 3: "+ ∫위치", 4: "+ ∫기울기"}


def main(argv=None) -> int:
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("--stage", type=int, nargs="*", choices=sorted(MODULES), help="stages to run (default: all, in order)")
    ap.add_argument("--keep-going", action="store_true", help="do not stop at the first failing stage")
    ap.add_argument("--out", type=Path, default=HERE / "results", help="report folder")
    args = ap.parse_args(argv)
    stages = sorted(args.stage) if args.stage else sorted(MODULES)
    summary = []
    t0 = time.time()
    for n in stages:
        print(f"\n===== 단계 {n}: {TITLES[n]} =====")
        suite = unittest.defaultTestLoader.loadTestsFromName(MODULES[n])
        t1 = time.time()
        res = unittest.TextTestRunner(verbosity=2).run(suite)
        ok = res.wasSuccessful()
        summary.append(dict(stage=n, title=TITLES[n], passed=ok, tests=res.testsRun, failures=len(res.failures), errors=len(res.errors),
                            seconds=round(time.time() - t1, 1)))
        if not ok and not args.keep_going:
            print(f"\n단계 {n} 실패: 이후 단계는 실행하지 않았습니다(--keep-going으로 계속 실행).")
            break
    args.out.mkdir(parents=True, exist_ok=True)
    head = ["# 단계별 통과 시험 요약", "", "| 단계 | 내용 | 판정 | 시험 수 | 실패 | 오류 | 시간 [s] |", "|---|---|---|---|---|---|---|"]
    for s in summary:
        head.append(f"| {s['stage']} | {s['title']} | {'통과' if s['passed'] else '**실패**'} | {s['tests']} | {s['failures']} | {s['errors']} | {s['seconds']} |")
    not_run = [n for n in stages if n not in [s["stage"] for s in summary]]
    if not_run:
        head.append("")
        head.append("실행하지 않은 단계: " + ", ".join(str(n) for n in not_run))
    md = "\n".join(head) + "\n\n" + SC.report_markdown().split("\n", 2)[2]
    (args.out / "stage_report.md").write_text(md, encoding="utf-8")
    (args.out / "stage_report.json").write_text(json.dumps(dict(summary=summary, records=SC.RECORDS), indent=1, default=float, ensure_ascii=False),
                                                encoding="utf-8")
    print(f"\n보고서: {args.out / 'stage_report.md'}  (전체 {time.time() - t0:.0f} s)")
    for s in summary:
        print(f"  단계 {s['stage']} {s['title']}: {'통과' if s['passed'] else '실패'}")
    return 0 if all(s["passed"] for s in summary) and not not_run else 1


if __name__ == "__main__":
    sys.exit(main())
