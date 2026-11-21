#!/usr/bin/env bash
# User-operated, bounded RViz diagnosis; not a supervised campaign run.
set -euo pipefail
REPO=$(cd "$(dirname "${BASH_SOURCE[0]}")/../.." && pwd)
INSIDE=0
if [[ ${1:-} == --inside ]]; then INSIDE=1; shift; fi
MODE=A
RATE=0.1
DURATION=10
while (( $# )); do
    case "$1" in
        --mode) MODE=${2:?}; shift 2 ;;
        --rate) RATE=${2:?}; shift 2 ;;
        --duration) DURATION=${2:?}; shift 2 ;;
        -h|--help) echo "Usage: bash $0 [--mode A|B|C] [--rate 0.1] [--duration 10]"; exit 0 ;;
        *) echo "Unknown argument: $1" >&2; exit 2 ;;
    esac
done
[[ $MODE =~ ^[ABC]$ ]] || { echo 'Mode must be A, B or C' >&2; exit 2; }
WALL=$(python3 - "$RATE" "$DURATION" <<'PY'
import math,sys
r,d=map(float,sys.argv[1:])
if not (math.isfinite(r) and math.isfinite(d) and 0<r<=1 and 0<d<=60):
    raise SystemExit('Require 0 < rate <= 1 and 0 < duration <= 60 seconds (short diagnosis only).')
print(math.ceil(d/r*2+60))
PY
)
if (( ! INSIDE )) && [[ ! -f /.dockerenv ]]; then
    export FAST_LIVO2_GO2W_IMAGE=sha256:4a7f294e39a8ad5dbe23223ceb0124752e52c8caa473c5253a2c05999f764a47
    export FAST_LIVO2_GO2W_CONTAINER=baseline-rviz-$$
    export ROS_DOMAIN_ID=89
    exec bash "$REPO/docker/robot/run.sh" bash /external/scripts/fast_livo2/reconstruct_baseline_rviz.sh --inside --mode "$MODE" --rate "$RATE" --duration "$DURATION"
fi
set +u
source /opt/ros/humble/setup.bash
set -u
PKG="$REPO/experiments/unified_acquisition/frozen_20261111_swap_telemetry"
OUT=$(mktemp -d "$REPO/experiments/unified_acquisition/manual_baseline_${MODE}_$(date +%Y%m%d_%H%M%S)_XXXXXX")
python3 - "$PKG/params/baseline_${MODE}.yaml" "$OUT/params.yaml" "$RATE" "$DURATION" <<'PY'
import sys,yaml
from pathlib import Path
src,dst,r,d=sys.argv[1:]
p=yaml.safe_load(Path(src).read_text())
p['laserMapping']['ros__parameters']['offline'].update(rate=float(r),duration=float(d))
Path(dst).write_text(yaml.safe_dump(p,sort_keys=False))
PY
export LD_LIBRARY_PATH="$PKG/livox/lib:${LD_LIBRARY_PATH:-}"
export ROS_DOMAIN_ID=89 ROS_LOCALHOST_ONLY=1 RMW_IMPLEMENTATION=rmw_cyclonedds_cpp
export CYCLONEDDS_URI='<CycloneDDS><Domain><General><AllowMulticast>false</AllowMulticast></General><Discovery><ParticipantIndex>auto</ParticipantIndex><Peers><Peer Address="127.0.0.1"/></Peers></Discovery></Domain></CycloneDDS>'
export FAST_LIVO2_DETERMINISTIC=1 OMP_NUM_THREADS=1 OPENBLAS_NUM_THREADS=1 MKL_NUM_THREADS=1
unset EXPERIMENT_VISUAL_OFF
if [[ $MODE == B ]]; then export EXPERIMENT_VISUAL_OFF=1; fi
sha256sum "$PKG/build/baseline/fastlivo2_offline" > "$OUT/binary.sha256"
printf 'mode=%s\nrate=%s\nduration=%s\nwall_limit=%s\npackage=%s\n' "$MODE" "$RATE" "$DURATION" "$WALL" "$PKG" > "$OUT/manual_run.txt"
rviz2 -d "$REPO/config/fast_livo2/fast_livo2.rviz" --ros-args -p use_sim_time:=false > "$OUT/rviz.log" 2>&1 &
RVIZ_PID=$!
cleanup() { kill "$RVIZ_PID" 2>/dev/null || true; }
trap cleanup EXIT
printf 'Output: %s\nManual diagnostic: campaign divergence/resource supervisor is not active.\n' "$OUT"
sleep 3
CMD=(timeout --signal=INT --kill-after=5s "${WALL}s" "$PKG/build/baseline/fastlivo2_offline" "$REPO/bags/raw_20260907_114335" "$OUT/frontend" --ros-args --params-file "$OUT/params.yaml")
printf '%q ' "${CMD[@]}" > "$OUT/command.sh"
printf '\n' >> "$OUT/command.sh"
set +e
"${CMD[@]}" 2>&1 | tee "$OUT/frontend.log"
STATUS=${PIPESTATUS[0]}
set -e
printf '%s\n' "$STATUS" > "$OUT/exit_code.txt"
printf 'Estimator exit: %s. Close RViz to finish. Output: %s\n' "$STATUS" "$OUT"
wait "$RVIZ_PID" || true
exit "$STATUS"
