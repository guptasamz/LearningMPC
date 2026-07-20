#!/usr/bin/env bash
# Friction (mu) sweep campaign for the LEARNT dynamics model
# (--dynamics residual: kinematic nominal + online error regression):
#   10 mu values x 3 maps x 100 laps, SPEED_MAX = 20 m/s.
#   Same run parameters as run_friction_campaign.sh (the known-dynamics
#   campaign), including the per-cell safe-set/margin fixes.
# Storage: results_residual_dynmics/<map>/<map>_<mu>/
set -u
HERE="$(cd "$(dirname "$0")" && pwd)"
PY="$HERE/../.venv/bin/python"
VMAX=20
MUS="0.1 0.2 0.3 0.4 0.5 0.6 0.7 0.8 0.9 1.0"
LAPS=100
SIMCAP=25000

# Per-(map, mu) extra flags — identical to the known-dynamics campaign.
cell_flags () {
  local name="$1" mu="$2"
  case "$name:$mu" in
    levinelobby_track:0.1)
      echo "--ss-file $HERE/../data/levinelobby_ss_mu0.1.csv";;
    barc_oval_orginal:0.1)
      echo "--ss-file $HERE/../data/maps/barc_oval_orginal/barc_oval_orginal_ss_mu0.1.csv --map-margin 0.30";;
  esac
}

run_map () {
  local name="$1"; shift
  local extra=("$@")
  local base="$HERE/results_residual_dynmics/${name}"
  mkdir -p "$base"
  echo "[campaign] === MAP $name start $(date '+%H:%M:%S') ==="
  for mu in $MUS; do
    local out="$base/${name}_${mu}"
    mkdir -p "$out"
    nohup "$PY" "$HERE/lmpc_gym.py" "${extra[@]}" $(cell_flags "$name" "$mu") \
        --dynamics residual \
        --laps $LAPS --speed-max $VMAX --mu $mu --max-sim-time $SIMCAP \
        --out "$out" > "$out/run.log" 2>&1 &
  done
  wait
  echo "[campaign] === MAP $name done $(date '+%H:%M:%S') ==="
  for mu in $MUS; do
    local out="$base/${name}_${mu}"
    echo "[campaign] $name mu=$mu :: $(grep -E 'laps completed' "$out/run.log" | tail -1)"
  done
}

run_map barc_oval          --map barc_oval
run_map barc_oval_orginal  --map barc_oval_orginal --map-margin 0.25
run_map levinelobby_track  # default map path (no --map flag)

echo "[campaign] ALL MAPS DONE $(date '+%H:%M:%S')"
