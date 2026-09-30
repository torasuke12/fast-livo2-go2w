from pathlib import Path
import subprocess, shlex, difflib
root=Path('/external/humble_ws/src/fast-livo2'); build=Path('/external/humble_ws/build/fast_livo2'); out=Path('/validation')
s=(root/'src/voxel_map.cpp').read_text(); original=s
s=s.replace('#include "voxel_map.h"','#include "voxel_map.h"\n#include "observer.h"',1)
needle='    if (EKF_stop_flg) break;'
assert s.count(needle)==1
s=s.replace(needle,'''    observeLio(scan_time, iterCount, EKF_stop_flg, config_setting_.xicp_.enabled,
               Hsub, R_inv, meas_vec, vec, solution, iteration_covariance, state_.cov,
               xicp_update.gain,
               xicp_detection.constraints.rows() ? xicp_detection.constraints *
                 fast_livo2::xicp::localPoseJacobian(before_update.rot_end, extR_, extT_, DIM_STATE) : Eigen::MatrixXd(0, DIM_STATE),
               xicp_detection.target_gain);
'''+needle)
(out/'voxel_map.cpp').write_text(s)
(out/'observer.patch').write_text(''.join(difflib.unified_diff(original.splitlines(True),s.splitlines(True),fromfile='src/voxel_map.cpp',tofile='src/voxel_map.cpp')))
flags={}
for line in (build/'CMakeFiles/fast_livo2_lio.dir/flags.make').read_text().splitlines():
    if ' = ' in line:
        k,v=line.split(' = ',1); flags[k]=shlex.split(v)
command=['/usr/bin/c++',*flags['CXX_DEFINES'],*flags['CXX_INCLUDES'],*flags['CXX_FLAGS'],'-I/validation']
for name in ['voxel_map','observer']:
    print('Compiling',name,flush=True)
    subprocess.run([*command,'-c',str(out/(name+'.cpp')),'-o',str(out/(name+'.cpp.o'))],check=True)
archive=out/'libfast_livo2_lio_observer.a'
objects=[out/'voxel_map.cpp.o',out/'observer.cpp.o',build/'CMakeFiles/fast_livo2_lio.dir/src/voxel_map_initialization.cpp.o',build/'CMakeFiles/fast_livo2_lio.dir/src/voxel_map_xicp.cpp.o',build/'CMakeFiles/fast_livo2_lio.dir/include/ikd-Tree/ikd_Tree.cpp.o']
subprocess.run(['/usr/bin/ar','rcs',str(archive),*map(str,objects)],check=True)
cmd=shlex.split((build/'CMakeFiles/fastlivo2_mapping.dir/link.txt').read_text())
cmd[cmd.index('-o')+1]=str(out/'fastlivo2_observer')
cmd=[str(archive) if x=='libfast_livo2_lio.a' else x for x in cmd]
assert str(archive) in cmd
subprocess.run(cmd,cwd=build,check=True)
print('Built',out/'fastlivo2_observer',flush=True)
