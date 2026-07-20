#!/usr/bin/env bash
# CRC (control-rate cost, r_d) sweep for the learnt-dynamics LMPC:
#   CRC in {2.0, 1.5, 1.0, 0.5, 0.1, 0.05, 0.01} x mu in {0.1..1.0}
#   map: $1 (default levinelobby_track); all other parameters = final
#   residual campaign (warm-start, per-map margins/safe-sets, 100 laps).
# Storage: results_residual_dynmics/CRC_experiments/<map>/<map>_<mu>/<map>_<mu>_<crc>/
set -u
HERE="$(cd "$(dirname "$0")" && pwd)"
PY="$HERE/../.venv/bin/python"
WS="$HERE/../online_training/data/warmstart"
MAP="${1:-levinelobby_track}"
CRCS="2.0 1.5 1.0 0.5 0.1 0.05 0.01"
MUS="0.1 0.2 0.3 0.4 0.5 0.6 0.7 0.8 0.9 1.0"
BASE="$HERE/results_residual_dynmics/CRC_experiments/$MAP"

# per-map flags, matching run_final_residual_campaign.sh
map_flags () {
  case "$MAP" in
    levinelobby_track) echo "";;
    barc_oval_orginal) echo "--map barc_oval_orginal --map-margin 0.25";;
    *) echo "--map $MAP";;
  esac
}
cell_flags () {
  local mu="$1"
  case "$MAP:$mu" in
    levinelobby_track:0.1)
      echo "--ss-file $HERE/../data/levinelobby_ss_mu0.1.csv";;
    barc_oval_orginal:0.1)
      echo "--ss-file $HERE/../data/maps/barc_oval_orginal/barc_oval_orginal_ss_mu0.1.csv --map-margin 0.30";;
  esac
}
ws_file () {
  local mu="$1"
  if [ "$MAP" = "barc_oval_orginal" ]; then
    echo "$WS/barc_orginal_dyn_mu${mu}.csv"
  else
    echo "$WS/${MAP}_dyn_mu${mu}.csv"
  fi
}

for crc in $CRCS; do
  echo "[crc] === $MAP CRC=$crc start $(date '+%H:%M:%S') ==="
  for mu in $MUS; do
    out="$BASE/${MAP}_${mu}/${MAP}_${mu}_${crc}"
    mkdir -p "$out"
    nohup "$PY" "$HERE/lmpc_gym.py" $(map_flags) $(cell_flags "$mu") \
        --dynamics residual --rd $crc \
        --reg-warmstart "$(ws_file "$mu")" \
        --laps 100 --speed-max 20 --mu $mu --max-sim-time 25000 \
        --out "$out" > "$out/run.log" 2>&1 &
  done
  wait
  for mu in $MUS; do
    out="$BASE/${MAP}_${mu}/${MAP}_${mu}_${crc}"
    echo "[crc] CRC=$crc mu=$mu :: $(grep -E 'laps completed' "$out/run.log" | tail -1) $(grep -oE 'best: [0-9.]+s' "$out/run.log" | tail -1)"
  done
done
echo "[crc] ALL DONE $(date '+%H:%M:%S')"
