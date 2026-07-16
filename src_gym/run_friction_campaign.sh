#!/usr/bin/env bash
# Friction (plant-mu) sweep campaign:
#   10 gym-plant mu values x 5 maps x 100 laps, SPEED_MAX = 20 m/s.
#   Controller model keeps friction_coeff from Lmpc_params.yaml (mismatch study).
#   Map-sequential: all 10 friction runs of one map run in parallel, the
#   campaign waits, then moves to the next map.
# Storage: results/<map>_<vmax>/<map>_<vmax>_<mu>/
set -u
HERE="$(cd "$(dirname "$0")" && pwd)"
PY="$HERE/../.venv/bin/python"
VMAX=20
MUS="0.1 0.2 0.3 0.4 0.5 0.6 0.7 0.8 0.9 1.0"
LAPS=100
SIMCAP=25000

# Per-(map, mu) extra flags. Low-mu cells need an initial safe set recorded at
# that mu (the default sets were driven at 1.5 m/s under mu=1.2 physics, which
# ice grip cannot follow), and barc_oval_orginal's mu=0.1 additionally needs
# margin 0.30. Recreate the safe sets with:
#   record_initial_ss.py --map levinelobby_track --mu 0.1 --v-target 0.8 \
#       --out-ss ../data/levinelobby_ss_mu0.1.csv
#   record_initial_ss.py --map barc_oval_orginal --mu 0.1 --v-target 0.7 \
#       --out-ss ../data/maps/barc_oval_orginal/barc_oval_orginal_ss_mu0.1.csv
# (a later --map-margin overrides the earlier one: argparse last-wins)
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
  local name="$1"; shift        # storage name
  local extra=("$@")            # extra runner flags (--map X / --map-margin Y)
  local base="$HERE/results/${name}_${VMAX}"
  mkdir -p "$base"
  echo "[campaign] === MAP $name start $(date '+%H:%M:%S') ==="
  for mu in $MUS; do
    local out="$base/${name}_${VMAX}_${mu}"
    mkdir -p "$out"
    nohup "$PY" "$HERE/lmpc_gym.py" "${extra[@]}" $(cell_flags "$name" "$mu") \
        --laps $LAPS --speed-max $VMAX --mu $mu --max-sim-time $SIMCAP \
        --out "$out" > "$out/run.log" 2>&1 &
  done
  wait
  echo "[campaign] === MAP $name done $(date '+%H:%M:%S') ==="
  for mu in $MUS; do
    local out="$base/${name}_${VMAX}_${mu}"
    echo "[campaign] $name mu=$mu :: $(grep -E 'laps completed' "$out/run.log" | tail -1)"
  done
}

run_map barc_oval          --map barc_oval
run_map barc_oval_orginal  --map barc_oval_orginal --map-margin 0.25
run_map levinelobby_track  # default map path (no --map flag)
run_map Sepang             --map Sepang
run_map YasMarina          --map YasMarina

echo "[campaign] ALL MAPS DONE $(date '+%H:%M:%S')"


# MAP_MARGIN up for Sepang runs — wide track (1.1 m half-widths) affords 0.6–0.7; directly absorbs the slide
# Higher CRC (we showed 0.5–1.0 survives at the limit on levinelobby)
# Higher q_s