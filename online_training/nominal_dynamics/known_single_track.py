"""Known dynamics baseline: the single-track model the current LMPC uses.

Verbatim port of the `use_dyn` branch of get_linearized_dynamics() in
src_gym/cpp/lmpc_core.cpp (itself from mlab-upenn/LearningMPC), reduced to
the velocity subsystem (v, omega, beta) and integrated with RK4 at Ts.

The gym plant integrates the same equations (plus steering-servo and
acceleration limits), so at matched friction this baseline should be
near-exact — it is the reference the learnt model is compared against.
"""

import numpy as np

G = 9.81


class KnownSingleTrack:
    def __init__(self, params, Ts):
        """params: dict with the Lmpc_params.yaml keys (l_cg2front, l_cg2rear,
        height_cg, C_S_front, C_S_rear, moment_inertia, mass, friction_coeff,
        wheelbase)."""
        self.l_f = float(params["l_cg2front"])
        self.l_r = float(params["l_cg2rear"])
        self.h = float(params["height_cg"])
        self.cs_f = float(params["C_S_front"])
        self.cs_r = float(params["C_S_rear"])
        self.I_z = float(params["moment_inertia"])
        self.m = float(params["mass"])
        self.mu = float(params["friction_coeff"])
        self.wb = float(params["wheelbase"])  # used as l_f+l_r, as in LMPC.cpp
        self.Ts = float(Ts)

    def _deriv(self, v, omega, beta, a, delta):
        rear_val = G * self.l_r - a * self.h
        front_val = G * self.l_f + a * self.h
        v_safe = np.maximum(v, 0.1)  # model is singular at v=0

        v_dot = a
        omega_dot = (self.mu * self.m / (self.I_z * self.wb)) * (
            self.l_f * self.cs_f * delta * rear_val
            + beta * (self.l_r * self.cs_r * front_val
                      - self.l_f * self.cs_f * rear_val)
            - (omega / v_safe) * (self.l_f ** 2 * self.cs_f * rear_val
                                  + self.l_r ** 2 * self.cs_r * front_val))
        beta_dot = (self.mu / (v_safe * (self.l_r + self.l_f))) * (
            self.cs_f * delta * rear_val
            - beta * (self.cs_r * front_val + self.cs_f * rear_val)
            + (omega / v_safe) * (self.cs_r * self.l_r * front_val
                                  - self.cs_f * self.l_f * rear_val)) - omega
        return v_dot, omega_dot, beta_dot

    def predict_velocities(self, vx, vy, omega, a, delta):
        """One-step RK4 prediction of (v_x, v_y, omega). Vectorized."""
        v = np.hypot(vx, vy)
        beta = np.arctan2(vy, np.maximum(vx, 1e-6))
        s = np.array([v, omega, beta], dtype=float)

        def f(state):
            return np.array(self._deriv(state[0], state[1], state[2], a, delta))

        h = self.Ts
        k1 = f(s)
        k2 = f(s + 0.5 * h * k1)
        k3 = f(s + 0.5 * h * k2)
        k4 = f(s + h * k3)
        v1, om1, b1 = s + (h / 6.0) * (k1 + 2 * k2 + 2 * k3 + k4)
        return v1 * np.cos(b1), v1 * np.sin(b1), om1

    def predict_velocities_stacked(self, X, U):
        vx, vy, om = self.predict_velocities(
            X[:, 0], X[:, 1], X[:, 2], U[:, 0], U[:, 1])
        return np.stack([vx, vy, om], axis=1)
