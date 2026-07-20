"""Verify the C++ residual regressor (residual_core) against the numpy
reference (residual_learnt_dynamics/error_regression.py).

Both are driven through the IDENTICAL rolling protocol used by
offline_eval.py: buffer = laps < L, predict lap L, add lap L. Agreement is
checked element-wise on the predictions (and on linearize() corrections at
sample points). The neighbor sets are identical by construction (same metric,
brute force vs exact kd-tree), so any disagreement beyond accumulation-order
noise (~1e-10) is a port bug.

Usage:
    python verify_cpp_port.py [steps_csv] [--test-laps 20]
"""

import argparse
import glob
import os
import sys
import time

import numpy as np
import pandas as pd
import yaml

from nominal_dynamics.kinematic_bicycle import KinematicBicycleNominal
from residual_learnt_dynamics.error_regression import ErrorDynamicsRegressor
from offline_eval import load_pairs, TS, STATES

import residual_core

HERE = os.path.dirname(os.path.abspath(__file__))
PARAMS_YAML = os.path.join(HERE, "..", "Lmpc_params.yaml")
TOL = 1e-8


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("steps_csv", nargs="?", default=None)
    ap.add_argument("--test-laps", type=int, default=20)
    ap.add_argument("--dist-max", type=float, default=2.0)
    ap.add_argument("--k-max", type=int, default=256)
    args = ap.parse_args()

    path = args.steps_csv or sorted(
        glob.glob(os.path.join(HERE, "data", "*", "steps_*.csv")))[-1]
    print(f"data: {path}")
    with open(PARAMS_YAML) as f:
        p = yaml.safe_load(f)
    l_f, l_r = p["l_cg2front"], p["l_cg2rear"]

    X, U, Xn, lap = load_pairs(path)
    laps = np.unique(lap)
    split = laps[-args.test_laps]
    tr = lap < split

    nominal = KinematicBicycleNominal(l_f, l_r, TS)
    py_reg = ErrorDynamicsRegressor(nominal, dist_max=args.dist_max,
                                    k_max=args.k_max)
    cpp_reg = residual_core.ErrorDynamicsRegressor(
        l_f=l_f, l_r=l_r, Ts=TS, dist_max=args.dist_max, k_max=args.k_max)

    py_reg.add_samples(X[tr], U[tr], Xn[tr])
    cpp_reg.add_samples(X[tr], U[tr], Xn[tr])

    # nominal model agreement first (isolates that part)
    dn = np.abs(nominal.predict_velocities_stacked(X, U)
                - cpp_reg.predict_nominal(X, U)).max()
    print(f"nominal prediction     max|py-cpp| = {dn:.3e}")

    max_pred = 0.0
    used_mismatch = 0
    n_q = 0
    t_py = t_cpp = 0.0
    for L in laps[laps >= split]:
        m = lap == L
        t0 = time.time(); py_p, py_u = py_reg.predict(X[m], U[m]); t_py += time.time() - t0
        t0 = time.time(); cp_p, cp_u = cpp_reg.predict(X[m], U[m]); t_cpp += time.time() - t0
        max_pred = max(max_pred, np.abs(py_p - cp_p).max())
        used_mismatch += int((py_u != cp_u).sum())
        n_q += m.sum()
        py_reg.add_samples(X[m], U[m], Xn[m])
        cpp_reg.add_samples(X[m], U[m], Xn[m])
    print(f"rolling predictions    max|py-cpp| = {max_pred:.3e}  "
          f"({n_q} queries; used-flag mismatches: {used_mismatch})")
    print(f"timing: numpy {t_py:.1f}s, C++ {t_cpp:.1f}s")

    # linearize() spot checks on a spread of test points
    rng = np.random.default_rng(0)
    pick = rng.choice(np.where(~tr)[0], size=50, replace=False)
    max_lin = 0.0
    for i in pick:
        pA, pB, pC = py_reg.linearize(X[i], U[i])
        cA, cB, cC = cpp_reg.linearize(X[i], U[i])
        max_lin = max(max_lin,
                      np.abs(pA - cA).max(), np.abs(pB - cB).max(),
                      np.abs(pC - cC).max())
    print(f"linearize (50 points)  max|py-cpp| = {max_lin:.3e}")

    ok = dn < TOL and max_pred < TOL and max_lin < TOL and used_mismatch == 0
    print(f"\n{'PASS' if ok else 'FAIL'} (tolerance {TOL:g})")
    sys.exit(0 if ok else 1)


if __name__ == "__main__":
    main()
