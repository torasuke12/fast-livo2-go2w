#!/usr/bin/env bash
# Chronological frontend only. The exporter never changes estimated poses.
set -euo pipefail
REPO=$(cd "$(dirname "${BASH_SOURCE[0]}")/../.." && pwd)
if [[ ${1:-} == --help || $# -lt 2 ]]; then
    echo "Usage: $0 BAG NEW_OUTPUT --params YAML [--lio-only|--livo] [--rate N] [--capture-diagnostics]"
    echo "       [--start-room A B] [--return-room A B]"
    echo "Default evaluation windows: 4–12 s and 632–648 s (raw_20260907_114335)."
    echo "Runs only chronological LiDAR/IMU/image estimation, then exports its unchanged trajectory and map."
    echo "--capture-diagnostics saves the source parameters, diagnostic status, and run outcome."
    echo "Temporal CSV logs are always saved; legacy LIO detail/geometry logs are enabled by this option."
    exit 0
fi
BAG=$(realpath -e "$1"); OUTPUT=$(realpath -m "$2"); shift 2
PARAMS=""; IMAGE=""; RATE=0; CAPTURE_DIAGNOSTICS=0; START=(4 12); RETURN=(632 648)
while [[ $# -gt 0 ]]; do
    case "$1" in
        --params) PARAMS=$(realpath -e "${2:?--params requires YAML}"); shift ;;
        --lio-only) IMAGE=0 ;;
        --livo) IMAGE=1 ;;
        --rate) RATE="${2:?--rate requires a value}"; shift ;;
        --capture-diagnostics) CAPTURE_DIAGNOSTICS=1 ;;
        --start-room) START=("${2:?}" "${3:?}"); shift 2 ;;
        --return-room) RETURN=("${2:?}" "${3:?}"); shift 2 ;;
        *) echo "Unknown option: $1" >&2; exit 2 ;;
    esac
    shift
done
[[ -n "$PARAMS" && -f "$BAG/metadata.yaml" ]] || { echo "--params and a valid bag are required." >&2; exit 2; }
[[ ! -e "$OUTPUT" ]] || { echo "Use a new output directory: $OUTPUT" >&2; exit 2; }
PYTHON=${FAST_LIVO2_MAP_PYTHON:-$REPO/.venv-reconstruction/bin/python}
[[ -x "$PYTHON" ]] || { echo "Install reconstruction-requirements.txt in .venv-reconstruction, or set FAST_LIVO2_MAP_PYTHON." >&2; exit 2; }
"$PYTHON" -c 'import numpy, scipy, open3d, matplotlib, yaml'
BINARY="$REPO/humble_ws/install/fast_livo2/lib/fast_livo2/fastlivo2_offline"
DIAGNOSTIC_OUTPUT="$OUTPUT/frontend/diagnostics"
if [[ ! -f /opt/ros/humble/setup.bash || ! -x "$BINARY" ]]; then
    DIAGNOSTIC_OUTPUT=/result/frontend/diagnostics
fi
mkdir -p "$OUTPUT"
CAPTURE_PHASE=preparing
if [[ $CAPTURE_DIAGNOSTICS == 1 ]]; then
    capture_finish() {
        local status=$?
        trap - EXIT
        "$PYTHON" - "$OUTPUT" "$CAPTURE_PHASE" "$status" <<'PY' || true
import csv,json,pathlib,sys
out=pathlib.Path(sys.argv[1]);phase=sys.argv[2];status=int(sys.argv[3]);frontend=out/'frontend'
result={'exit_code':status,'last_phase':phase,'frontend_summary':None,'diagnostic_files':{},'problems':[]}
capture=out/'capture.json'
settings=json.loads(capture.read_text()) if capture.is_file() else {}
summary=frontend/'summary.json'
if summary.is_file():
    try: result['frontend_summary']=json.loads(summary.read_text())
    except (OSError,ValueError) as error: result['problems'].append(f'Invalid frontend summary: {error}')
else: result['problems'].append('Missing frontend summary (partial or failed replay)')
if status == 0 and result['frontend_summary'] and not (
        result['frontend_summary'].get('complete') and result['frontend_summary'].get('bag_exhausted')):
    result['problems'].append('Frontend did not record a completed full replay')
for name in ('mat_pre.txt','mat_out.txt','imu.txt','temporal_window.csv','temporal_states.csv',
             'temporal_initialization.csv','temporal_visual.csv'):
    path=frontend/'logs'/name
    if path.is_file():
        result['diagnostic_files'][name]={'bytes':path.stat().st_size}
        if name.endswith('.csv'):
            try:
                with path.open(newline='') as stream:
                    rows=csv.reader(stream)
                    next(rows,None)
                    result['diagnostic_files'][name]['data_rows']=sum(1 for row in rows if row)
            except (OSError,csv.Error) as error: result['problems'].append(f'Invalid {name}: {error}')
if status == 0:
    required=['mat_pre.txt','mat_out.txt','imu.txt']
    if settings.get('temporal_window'):
        required+=['temporal_window.csv','temporal_states.csv','temporal_initialization.csv']
        if settings.get('mode')=='livo': required.append('temporal_visual.csv')
    for name in required:
        if name not in result['diagnostic_files']:
            result['problems'].append(f'Missing diagnostic log: {name}')
    if settings.get('legacy_lio_diagnostics_enabled') and not list((frontend/'diagnostics').glob('run-*')):
        result['problems'].append('Missing detailed LIO diagnostic run')
if status: result['problems'].append(f'Reconstruction exited with status {status} during {phase}')
result['capture_integrity']='ok' if not result['problems'] else 'incomplete'
(out/'capture_summary.json').write_text(json.dumps(result,indent=2)+'\n')
PY
        return "$status"
    }
    trap capture_finish EXIT
fi
"$PYTHON" - "$PARAMS" "$OUTPUT" "$IMAGE" "$RATE" "$REPO" "$BAG" "$CAPTURE_DIAGNOSTICS" \
    "${START[@]}" "${RETURN[@]}" "$DIAGNOSTIC_OUTPUT" <<'PY'
import hashlib,json,math,pathlib,shutil,sys,yaml
source,out=map(pathlib.Path,sys.argv[1:3]);repo=pathlib.Path(sys.argv[5])
config=yaml.safe_load(source.read_text());p=config['laserMapping']['ros__parameters']
if sys.argv[3]:p['common']['img_en']=int(sys.argv[3])
rate=float(sys.argv[4])
if not math.isfinite(rate) or rate<0:raise ValueError('rate must be finite and nonnegative (0 = unpaced)')
p['offline']={'rate':rate,'duration':0.,'keyframe_interval':.5}
p['publish']['map_en']=False
p['evo']['pose_output_en']=False;p['pcd_save']['pcd_save_en']=False;p['image_save']['img_save_en']=False
capture=sys.argv[7]=='1';temporal=p['lio']['temporal']['enabled']
diagnostics=p['lio']['diagnostics']
diagnostics['enabled']=capture and not temporal
diagnostics['output_dir']=sys.argv[12]
if capture and not temporal:
    diagnostics['detail'].update(enabled=True,final_only=True,start_sec=0.,end_sec=360.)
    diagnostics['geometry']['enabled']=True
p['lio']['xicp']['diagnostics']=capture and not temporal and p['lio']['xicp']['enabled']
(out/'params.yaml').write_text(yaml.safe_dump(config,sort_keys=False))
if capture:
    shutil.copyfile(source,out/'original_params.yaml')
    (out/'capture.json').write_text(json.dumps({
        'bag':str(pathlib.Path(sys.argv[6]).resolve()),'source_params':str(source.resolve()),
        'mode':'livo' if p['common']['img_en'] else 'lio','input_rate':rate,
        'evaluation_windows_seconds':[list(map(float,sys.argv[8:10])),list(map(float,sys.argv[10:12]))],
        'temporal_window':bool(temporal),
        'legacy_lio_diagnostics_enabled':diagnostics['enabled'],
        'temporal_csv_logs_enabled':bool(temporal),
        'source_params_sha256':hashlib.sha256(source.read_bytes()).hexdigest(),
        'effective_params_sha256':hashlib.sha256((out/'params.yaml').read_bytes()).hexdigest(),
        'note':'Offline frontend logs are saved directly; no ROS bag-player or RViz monitor is used.'
    },indent=2)+'\n')
files=[*repo.glob('humble_ws/src/fast-livo2/src/*.cpp'),*repo.glob('humble_ws/src/fast-livo2/include/*.h'),
       repo/'humble_ws/src/fast-livo2/CMakeLists.txt',
       source,out/'params.yaml',repo/'scripts/fast_livo2/evaluate_online.py',repo/'scripts/fast_livo2/reconstruct_online.sh']
hashes={str(f):hashlib.sha256(f.read_bytes()).hexdigest() for f in files if f.is_file()}
(out/'provenance.json').write_text(json.dumps({'bag':sys.argv[6],'loop_constraints':0,'sha256':hashes},indent=2)+'\n')
PY
export FAST_LIVO2_DETERMINISTIC=1 OMP_NUM_THREADS=1 OPENBLAS_NUM_THREADS=1
CAPTURE_PHASE=running_frontend
if [[ -f /opt/ros/humble/setup.bash && -x "$BINARY" ]]; then
    set +u
    source /opt/ros/humble/setup.bash
    source "$REPO/humble_ws/install/setup.bash"
    set -u
    unset CYCLONEDDS_URI
    export ROS_DOMAIN_ID=${FAST_LIVO2_RECONSTRUCTION_DOMAIN_ID:-89} ROS_LOCALHOST_ONLY=1
    export ROS_LOG_DIR="$OUTPUT/ros"
    sha256sum "$BINARY" > "$OUTPUT/binary.sha256"
    "$BINARY" "$BAG" "$OUTPUT/frontend" --ros-args --params-file "$OUTPUT/params.yaml" --log-level error > "$OUTPUT/frontend.log" 2>&1
else
    docker run --rm --network none --user "$(id -u):$(id -g)" \
        --memory="${FAST_LIVO2_ONLINE_MEMORY:-8g}" --memory-swap="${FAST_LIVO2_ONLINE_MEMORY:-8g}" \
        -e FAST_LIVO2_DETERMINISTIC=1 -e OMP_NUM_THREADS=1 -e OPENBLAS_NUM_THREADS=1 \
        -e ROS_DOMAIN_ID=89 -e ROS_LOCALHOST_ONLY=1 -e ROS_LOG_DIR=/result/ros \
        -v "$REPO:/external:ro" -v "$BAG:/bag:ro" -v "$OUTPUT:/result" -w /external \
        "${FAST_LIVO2_GO2W_IMAGE:-fast-livo2-go2w:latest}" bash -c '
            source /opt/ros/humble/setup.bash
            source humble_ws/install/setup.bash
            unset CYCLONEDDS_URI
            sha256sum humble_ws/install/fast_livo2/lib/fast_livo2/fastlivo2_offline > /result/binary.sha256
            exec humble_ws/install/fast_livo2/lib/fast_livo2/fastlivo2_offline /bag /result/frontend \
                --ros-args --params-file /result/params.yaml --log-level error
        ' > "$OUTPUT/frontend.log" 2>&1
fi
CAPTURE_PHASE=evaluating
"$PYTHON" "$REPO/scripts/fast_livo2/evaluate_online.py" "$OUTPUT/frontend" "$OUTPUT/export" \
    --start "${START[@]}" --return-window "${RETURN[@]}" --map | tee "$OUTPUT/evaluation.log"
CAPTURE_PHASE=complete
echo "Online map: $OUTPUT/export/map.ply"
echo "Unmodified online trajectory: $OUTPUT/export/trajectory.tum"
