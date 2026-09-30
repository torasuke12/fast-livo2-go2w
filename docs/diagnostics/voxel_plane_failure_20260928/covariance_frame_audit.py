"""Independent finite-difference and frame-equivariance check; no estimator mutation."""
from pathlib import Path
import json
import numpy as np
from scipy.spatial.transform import Rotation
p=np.array([4.,1.,-.5]);R=Rotation.from_euler('zy',[170,21],degrees=True).as_matrix()
P=np.diag([.0004,.001,.002]);C=np.array([[0,-p[2],p[1]],[p[2],0,-p[0]],[-p[1],p[0],0.]])
J=-R@C;eps=1e-7
numeric=np.column_stack([(R@Rotation.from_rotvec(np.eye(3)[j]*eps).apply(p)-R@Rotation.from_rotvec(-np.eye(3)[j]*eps).apply(p))/(2*eps) for j in range(3)])
current=C@P@C.T;correct=J@P@J.T;Q=Rotation.from_euler('z',65,degrees=True).as_matrix()
result=dict(point_imu=p.tolist(),right_tangent_covariance=P.tolist(),correct_jacobian='-R * skew(p_imu)',
 finite_difference_jacobian_max_abs=float(abs(numeric-J).max()),
 current_formula_world_rotation_equivariance_error_fro=float(np.linalg.norm(current-Q@current@Q.T)),
 correct_formula_world_rotation_equivariance_error_fro=float(np.linalg.norm((-Q@R@C)@P@(-Q@R@C).T-Q@correct@Q.T)),
 scope='Synthetic algebra check of current rotation-covariance term; not a replay or estimate of causal effect on the U-turn.')
assert result['finite_difference_jacobian_max_abs']<1e-7
assert result['correct_formula_world_rotation_equivariance_error_fro']<1e-12
assert result['current_formula_world_rotation_equivariance_error_fro']>.001
Path(__file__).with_name('covariance-frame-audit.json').write_text(json.dumps(result,indent=2)+'\n')
print(json.dumps(result,indent=2))
