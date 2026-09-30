#!/usr/bin/env bash
# Complete a bag without playback losses, then rebuild its map from optimized poses.
set -euo pipefail
REPO=$(cd "$(dirname "${BASH_SOURCE[0]}")/../.." && pwd)
if [[ $# -lt 2 || ${1:-} == --help ]]; then
    echo "Usage: $0 BAG NEW_OUTPUT [--return-to-start] [--livo] [--rviz] [--rate N] [--temporal-window on|off] [--capture-diagnostics]"
    echo "Host dependencies: python3 -m venv .venv-reconstruction; .venv-reconstruction/bin/pip install -r scripts/fast_livo2/reconstruction-requirements.txt"
    exit 0
fi
BAG=$(realpath "$1")
OUTPUT=$(realpath -m "$2")
shift 2
IMAGE=0
RVIZ=0
RATE=0
TEMPORAL=on
DIAGNOSTICS=0
BACKEND_ARGS=()
while [[ $# -gt 0 ]]; do
    case "$1" in
        --livo) IMAGE=1 ;;
        --return-to-start) BACKEND_ARGS+=(--return-to-start) ;;
        --rviz) RVIZ=1 ;;
        --rate) RATE="${2:?--rate requires a value}"; shift ;;
        --temporal-window) TEMPORAL="${2:?--temporal-window requires on or off}"; shift ;;
        --capture-diagnostics) DIAGNOSTICS=1 ;;
        *) echo "Unknown option: $1" >&2; exit 2 ;;
    esac
    shift
done
[[ "$TEMPORAL" == on || "$TEMPORAL" == off ]] || { echo "Invalid temporal-window" >&2; exit 2; }
PYTHON=${FAST_LIVO2_MAP_PYTHON:-$REPO/.venv-reconstruction/bin/python}
if [[ "$RVIZ" == 1 ]]; then
    PYTHON=${FAST_LIVO2_MAP_PYTHON:-$REPO/.venv-reconstruction-rviz/bin/python}
    [[ -n "${DISPLAY:-}" ]] || { echo "RViz requires DISPLAY. Run from a graphical desktop terminal." >&2; exit 2; }
    [[ -f /opt/ros/humble/setup.bash ]] || { echo "Use docker/robot/run.sh for --rviz mode." >&2; exit 2; }
    set +u
    source /opt/ros/humble/setup.bash
    source "$REPO/humble_ws/install/setup.bash"
    set -u
fi
if [[ ! -x "$PYTHON" ]]; then
    echo "Install reconstruction-requirements.txt in $PYTHON's venv, or set FAST_LIVO2_MAP_PYTHON." >&2
    exit 2
fi
"$PYTHON" -c 'import numpy, scipy, open3d, yaml'
[[ -f "$BAG/metadata.yaml" ]] || { echo "Bag metadata is missing." >&2; exit 2; }
[[ ! -e "$OUTPUT" ]] || { echo "Output already exists: $OUTPUT" >&2; exit 2; }
mkdir -p "$OUTPUT"
"$PYTHON" - "$REPO" "$OUTPUT" "$IMAGE" "$BAG" "$TEMPORAL" "$RATE" "$DIAGNOSTICS" <<'PY'
import hashlib,json,math,pathlib,sys,yaml
repo,out=map(pathlib.Path,sys.argv[1:3])
config=yaml.safe_load((repo/'humble_ws/src/fast-livo2/config/go2w_fast_livo2.yaml').read_text())
p=config['laserMapping']['ros__parameters']
p['common']['img_en']=int(sys.argv[3])
p['lio']['temporal'].update(enabled=sys.argv[5]=='on',initialization_mode='legacy_voxel',window_sec=2.,max_nodes=6,iterations=3)
rate=float(sys.argv[6])
if not math.isfinite(rate) or rate<0: raise ValueError('rate must be finite and >= 0 (0 = unpaced)')
p['offline']={'rate':rate}
p['imu'].update(initial_rotation_cov=.0001,initial_position_cov=.0001,initial_velocity_cov=.001)
p['lio']['xicp'].update(enabled=False,diagnostics=False)
p['lio']['diagnostics']['enabled']=sys.argv[7]=='1' and sys.argv[5]=='off'
p['lio']['diagnostics']['output_dir']=str(out/'frontend'/'diagnostics')
p['publish']['map_en']=False
p['evo']['pose_output_en']=False
p['pcd_save']['pcd_save_en']=False
p['image_save']['img_save_en']=False
(out/'params.yaml').write_text(yaml.safe_dump(config,sort_keys=False))
sources=list((repo/'humble_ws/src/fast-livo2/src').glob('*.cpp'))+list((repo/'humble_ws/src/fast-livo2/include').glob('*.h'))
sources += [out/'params.yaml',repo/'scripts/fast_livo2/optimize_reconstruction.py']
manifest={str(f):hashlib.sha256(f.read_bytes()).hexdigest() for f in sources}
(out/'provenance.json').write_text(json.dumps(dict(bag=sys.argv[4],sha256=manifest),indent=2))
PY
export OMP_NUM_THREADS=1 OPENBLAS_NUM_THREADS=1 FAST_LIVO2_DETERMINISTIC=1
if [[ "$RVIZ" == 1 ]]; then
    unset CYCLONEDDS_URI
    export ROS_DOMAIN_ID=${FAST_LIVO2_RECONSTRUCTION_DOMAIN_ID:-42}
    export ROS_LOCALHOST_ONLY=1
    export ROS_LOG_DIR="$OUTPUT/ros"
    sha256sum "$REPO/humble_ws/install/fast_livo2/lib/fast_livo2/fastlivo2_offline" > "$OUTPUT/binary.sha256"
    exec "$PYTHON" "$REPO/scripts/fast_livo2/reconstruct_with_correction.py" "$BAG" "$OUTPUT" "${BACKEND_ARGS[@]}"
fi
if [[ "$REPO" == /external ]]; then
    set +u
    source /opt/ros/humble/setup.bash
    source "$REPO/humble_ws/install/setup.bash"
    set -u
    unset CYCLONEDDS_URI
    sha256sum "$REPO/humble_ws/install/fast_livo2/lib/fast_livo2/fastlivo2_offline" > "$OUTPUT/binary.sha256"
    "$REPO/humble_ws/install/fast_livo2/lib/fast_livo2/fastlivo2_offline" "$BAG" "$OUTPUT/frontend" \
        --ros-args --params-file "$OUTPUT/params.yaml" --log-level error > "$OUTPUT/frontend.log" 2>&1
else
    docker run --rm --network none --user "$(id -u):$(id -g)" \
        -e OMP_NUM_THREADS=1 -e FAST_LIVO2_DETERMINISTIC=1 -e ROS_DOMAIN_ID=89 -e ROS_LOG_DIR=/result/ros \
        -v "$REPO:/external:ro" -v "$BAG:/bag:ro" -v "$OUTPUT:/result" \
        -w /external "${FAST_LIVO2_DOCKER_IMAGE:-fast-livo2-go2w:latest}" bash -c '
            source /opt/ros/humble/setup.bash
            source humble_ws/install/setup.bash
            unset CYCLONEDDS_URI
            sha256sum humble_ws/install/fast_livo2/lib/fast_livo2/fastlivo2_offline > /result/binary.sha256
            humble_ws/install/fast_livo2/lib/fast_livo2/fastlivo2_offline /bag /result/frontend \
                --ros-args --params-file /result/params.yaml --log-level error
        ' > "$OUTPUT/frontend.log" 2>&1
fi
"$PYTHON" "$REPO/scripts/fast_livo2/optimize_reconstruction.py" "$OUTPUT/frontend" "$OUTPUT/optimized" \
    "${BACKEND_ARGS[@]}" | tee "$OUTPUT/optimization.log"
echo "Reconstruction: $OUTPUT/optimized/map.ply"
echo "Trajectory: $OUTPUT/optimized/trajectory.tum"
