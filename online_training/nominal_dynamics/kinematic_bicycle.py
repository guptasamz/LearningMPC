"""Nominal dynamics: kinematic bicycle (Xue et al., arXiv:2309.10716, Sec. III).

Continuous model (CG reference, slip from steering geometry only):

    x_dot   = v cos(psi + beta)
    y_dot   = v sin(psi + beta)
    psi_dot = v sin(beta) / l_r
    v_dot   = a
    beta    = atan(l_r tan(delta) / (l_f + l_r))

Deviation from the paper: our LMPC commands u = (a, delta) directly (no
steering-rate state), so delta is an algebraic input and beta is evaluated
from the commanded steering. The residual model is expected to absorb the
servo lag this ignores.

Only the velocity subsystem needs a model for learning: pose states integrate
exactly from velocities. This class therefore predicts the next body-frame
velocities (v_x, v_y, omega) over one discrete step Ts:

    v+     = v + a Ts,          v = hypot(v_x, v_y)
    beta+  = atan(l_r tan(delta) / L)
    v_x+   = v+ cos(beta+)
    v_y+   = v+ sin(beta+)
    omega+ = v+ sin(beta+) / l_r      (= v_y+ / l_r)
"""

import numpy as np


class KinematicBicycleNominal:
    def __init__(self, l_f, l_r, Ts):
        self.l_f = float(l_f)
        self.l_r = float(l_r)
        self.L = self.l_f + self.l_r
        self.Ts = float(Ts)

    def predict_velocities(self, vx, vy, omega, a, delta):
        """One-step prediction of (v_x, v_y, omega). Vectorized.

        Args are arrays of equal shape (or scalars); omega is unused by the
        kinematic model (kept in the signature so all models share it).
        Returns (vx_next, vy_next, omega_next).
        """
        v = np.hypot(vx, vy)
        v_next = v + a * self.Ts
        beta_next = np.arctan(self.l_r * np.tan(delta) / self.L)
        vx_next = v_next * np.cos(beta_next)
        vy_next = v_next * np.sin(beta_next)
        omega_next = v_next * np.sin(beta_next) / self.l_r
        return vx_next, vy_next, omega_next

    def predict_velocities_stacked(self, X, U):
        """X: (n,3) [vx, vy, omega]; U: (n,2) [a, delta] -> (n,3)."""
        vx, vy, om = self.predict_velocities(
            X[:, 0], X[:, 1], X[:, 2], U[:, 0], U[:, 1])
        return np.stack([vx, vy, om], axis=1)
