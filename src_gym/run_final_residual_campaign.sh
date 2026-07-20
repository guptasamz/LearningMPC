#!/usr/bin/env bash
# FINAL residual-dynamics campaign (the blessed config):
#   kinematic nominal + error regression, 3-lap pure-pursuit warm-start at
#   matched mu, r_d = 0.5, reg_buffer_max = 200k (compiled default).
#   10 mu x 5 maps x 100 laps, SPEED_MAX 20.
# Phase 1 records missing warm-start files (adaptive v-target: 1.5 -> 1.0 ->
# 0.7 -> 0.5 until the pure-pursuit recorder survives; mu 0.1 starts at 0.7).
# Storage: results_residual_dynmics/final/<map>/<map>_<mu>/
set -u
HERE="$(cd "$(dirname "$0")" && pwd)"
PY="$HERE/../.venv/bin/python"
WS="$HERE/../online_training/data/warmstart"
SCRATCH="${TMPDIR:-/tmp}"
VMAX=20
MUS="0.1 0.2 0.3 0.4 0.5 0.6 0.7 0.8 0.9 1.0"
LAPS=100
SIMCAP=25000
RD=0.5

mkdir -p "$WS"

# ---- per-(map, mu) extra flags: same fixes as the known campaign ----
cell_flags () {
  local name="$1" mu="$2"
  case "$name:$mu" in
    levinelobby_track:0.1)
      echo "--ss-file $HERE/../data/levinelobby_ss_mu0.1.csv";;
    barc_oval_orginal:0.1)
      echo "--ss-file $HERE/../data/maps/barc_oval_orginal/barc_oval_orginal_ss_mu0.1.csv --map-margin 0.30";;
  esac
}
map_flags () {  # base flags per map (empty for levinelobby = default map)
  local name="$1"
  case "$name" in
    levinelobby_track) echo "";;
    barc_oval_orginal) echo "--map barc_oval_orginal --map-margin 0.25";;
    *) echo "--map $name";;
  esac
}

# ---- phase 1: warm-start recordings (skip existing) ----
record_ws () {
  local map="$1" mu="$2"
  local out="$WS/${map}_dyn_mu${mu}.csv"
  [ -s "$out" ] && return 0
  local vts="1.5 1.0 0.7 0.5"
  [ "$mu" = "0.1" ] && vts="0.7 0.5 0.3"
  for vt in $vts; do
    if "$PY" "$HERE/record_initial_ss.py" --map "$map" --laps 3 --mu "$mu" \
        --v-target "$vt" --out-ss "$SCRATCH/ss_${map}_${mu}.csv" \
        --out-dyn "$out" > "$SCRATCH/ws_${map}_${mu}.log" 2>&1; then
      echo "[ws] $map mu=$mu recorded at v=$vt ($(wc -l < "$out" | tr -d ' ') pairs)"
      return 0
    fi
    rm -f "$out"
  done
  echo "[ws] $map mu=$mu FAILED at all v-targets (cell will run without warm-start)"
  return 1
}

# NOTE: barc_oval_orginal files already exist under barc_orginal_dyn_mu*.csv
for mu in $MUS; do
  [ -s "$WS/barc_oval_orginal_dyn_mu${mu}.csv" ] || \
    cp "$WS/barc_orginal_dyn_mu${mu}.csv" "$WS/barc_oval_orginal_dyn_mu${mu}.csv" 2>/dev/null
done

for map in barc_oval levinelobby_track Sepang YasMarina; do
  echo "[ws] === recording $map $(date '+%H:%M:%S') ==="
  for mu in $MUS; do record_ws "$map" "$mu" & done
  wait
done

# ---- phase 2: campaign ----
run_map () {
  local name="$1"
  local base="$HERE/results_residual_dynmics/final/${name}"
  mkdir -p "$base"
  echo "[campaign] === MAP $name start $(date '+%H:%M:%S') ==="
  for mu in $MUS; do
    local out="$base/${name}_${mu}"
    mkdir -p "$out"
    local wsf="$WS/${name}_dyn_mu${mu}.csv"
    local wsarg=""
    [ -s "$wsf" ] && wsarg="--reg-warmstart $wsf"
    nohup "$PY" "$HERE/lmpc_gym.py" $(map_flags "$name") $(cell_flags "$name" "$mu") \
        $wsarg --dynamics residual --rd $RD \
        --laps $LAPS --speed-max $VMAX --mu $mu --max-sim-time $SIMCAP \
        --out "$out" > "$out/run.log" 2>&1 &
  done
  wait
  echo "[campaign] === MAP $name done $(date '+%H:%M:%S') ==="
  for mu in $MUS; do
    local out="$base/${name}_${mu}"
    echo "[campaign] $name mu=$mu :: $(grep -E 'laps completed' "$out/run.log" | tail -1) $(grep -oE 'best: [0-9.]+s' "$out/run.log" | tail -1)"
  done
}

run_map barc_oval
run_map barc_oval_orginal
run_map levinelobby_track
run_map Sepang
run_map YasMarina

echo "[campaign] ALL MAPS DONE $(date '+%H:%M:%S')"
