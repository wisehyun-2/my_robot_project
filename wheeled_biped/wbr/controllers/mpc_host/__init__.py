"""Stage-2 host of Plan 4: MPC velocity-reference generator above the MCU's LQI6 (docs/plan4_mpc_host.md).

    model.py          prediction model: the LQI6 closed loop (plant linearisation + MCU gain table), lifting, reduction
    reference_mpc.py  MpcHost (HostEmulator replacement), ReferenceQP (qpmpc + DAQP / OSQP), limits, observer, fallback
    fallback.py       stage-1 ramp shaping (last-resort fallback)
    identify.py       model validation against the simulation -> models/upkie_plan4/plan4_closed_loop.json, figures
    compare_hosts.py  stage-1 ramp host vs MPC host (velocity step) -> results/plan4_mpc/ramp_vs_mpc.png
    regression.py     criterion 20: stage-1 vs stage-2 acceptance summaries -> results/plan4_mpc/regression.md
"""
