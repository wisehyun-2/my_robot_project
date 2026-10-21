"""Port of upkie's MPCBalancer (upkie/controllers/mpc_balancer.py, Apache-2.0).

Same linear MPC on the wheeled inverted pendulum (qpmpc), but solved with an open-source QP
solver available on this platform (daqp / osqp / quadprog through qpsolvers) instead of ProxQP.
"""
from __future__ import annotations

import numpy as np
import qpsolvers
from qpmpc import MPCProblem, MPCQP, Plan
from qpmpc.systems import WheeledInvertedPendulum

from wbr.common.filters import clamp_abs, low_pass_filter


def get_target_states(pendulum: WheeledInvertedPendulum, state: np.ndarray, target_ground_velocity: float):
    nx = pendulum.STATE_DIM
    T = pendulum.sampling_period
    target_states = np.zeros((pendulum.nb_timesteps + 1) * nx)
    for k in range(pendulum.nb_timesteps + 1):
        target_states[k * nx] = state[0] + (k * T) * target_ground_velocity
        target_states[k * nx + 2] = target_ground_velocity
    return target_states


class MPCBalancer:
    def __init__(self, fall_pitch: float = 1.0, leg_length: float = 0.58, max_ground_accel: float = 10.0,
                 max_ground_velocity: float = 3.0, nb_timesteps: int = 50, sampling_period: float = 0.02,
                 stage_input_cost_weight: float = 1e-3, stage_state_cost_weight: float = 1e-3,
                 terminal_cost_weight: float = 1.0, solver: str = "daqp"):
        pendulum = WheeledInvertedPendulum(length=leg_length, max_ground_accel=max_ground_accel,
                                           nb_timesteps=nb_timesteps, sampling_period=sampling_period)
        mpc_problem = pendulum.build_mpc_problem(terminal_cost_weight=terminal_cost_weight,
                                                 stage_state_cost_weight=stage_state_cost_weight,
                                                 stage_input_cost_weight=stage_input_cost_weight)
        mpc_problem.initial_state = np.zeros(pendulum.STATE_DIM)
        mpc_problem.update_goal_state(np.zeros(pendulum.STATE_DIM))
        mpc_problem.update_target_states(np.zeros(nb_timesteps * pendulum.STATE_DIM))
        self.mpc_problem = mpc_problem
        self.mpc_qp = MPCQP(mpc_problem)
        self.pendulum = pendulum
        self.solver = solver
        self.fall_pitch = fall_pitch
        self.max_ground_velocity = max_ground_velocity
        self.commanded_velocity = 0.0
        self.fallen = False
        self.solve_failures = 0
        self.last_solve_time = 0.0

    def reset(self) -> None:
        self.commanded_velocity = 0.0
        self.fallen = False
        self.mpc_problem.initial_state = np.zeros(4)

    def step(self, target_ground_velocity: float, observation: dict, dt: float) -> float:
        import time as _time
        floor_contact = observation["floor_contact"]["contact"]
        base_pitch = observation["base_orientation"]["pitch"]
        base_angular_velocity = observation["base_orientation"]["angular_velocity"][1]
        ground_position = observation["wheel_odometry"]["position"]
        ground_velocity = observation["wheel_odometry"]["velocity"]
        self.fallen = abs(base_pitch) > self.fall_pitch
        cur_state = np.array([ground_position, base_pitch, ground_velocity, base_angular_velocity])
        nx = self.pendulum.STATE_DIM
        target_states = get_target_states(self.pendulum, cur_state, target_ground_velocity)
        self.mpc_problem.update_initial_state(cur_state)
        self.mpc_problem.update_goal_state(target_states[-nx:])
        self.mpc_problem.update_target_states(target_states[:-nx])
        self.mpc_qp.update_cost_vector(self.mpc_problem)
        t0 = _time.perf_counter()
        qpsol = qpsolvers.solve_problem(self.mpc_qp.problem, solver=self.solver)
        self.last_solve_time = _time.perf_counter() - t0
        plan = Plan(self.mpc_problem, qpsol)
        if self.fallen or not floor_contact:
            self.commanded_velocity = low_pass_filter(self.commanded_velocity, 0.1, 0.0, dt)
        elif plan.is_empty:
            self.solve_failures += 1
        else:
            self.pendulum.state = cur_state
            commanded_accel = plan.first_input[0]
            self.commanded_velocity = clamp_abs(self.commanded_velocity + commanded_accel * dt / 2.0,
                                                self.max_ground_velocity)
        return self.commanded_velocity
