from pathlib import Path
import argparse, json
import numpy as np
p=argparse.ArgumentParser(); p.add_argument('csv',type=Path); p.add_argument('--out',type=Path,required=True); p.add_argument('--xicp',type=Path); a=p.parse_args()
# All arrays are row-major. Pose columns: right IMU rotation [rad], world position [m].
with a.csv.open() as source:
    names=source.readline().strip().split(',')
    values=np.loadtxt(source,delimiter=',',ndmin=2)
d={name:values[:,i] for i,name in enumerate(names)}
def matrix(name,rows,cols):
    return np.stack([d[f'{name}_{i}_{j}'] for i in range(rows) for j in range(cols)],axis=-1).reshape(-1,rows,cols)
A=matrix('HWH',6,6); P=matrix('Piter',19,19); Ppost=matrix('Pcommitted',19,19)
KH=matrix('KH',19,6); Ki=matrix('Kinnov',19,1)[...,0]; K0i=matrix('K0innov',19,1)[...,0]
sol=matrix('solution',19,1)[...,0]; prior_delta=matrix('prior_delta',19,1)[...,0]
noise=matrix('gain_noise_diag',19,1)[...,0]; C=matrix('C',6,19); target=matrix('target',6,1)[...,0]
final=d['final']==1
assert np.isfinite(values).all(), 'Non-finite observer output'
print('rows',len(values),'scans',final.sum(),'time',d['time'][[0,-1]],'finite',True)
print('constraint correction error',np.max(np.linalg.norm(np.einsum('nij,nj->ni',C,sol-prior_delta)-target,axis=1)))
print('correction identity error',np.max(np.abs(sol-prior_delta-Ki)))
# Normalize columns to unit weighted norm before comparing coupled eigenvalues.
scale=1/np.sqrt(np.maximum(np.diagonal(A,axis1=1,axis2=2),1e-30))
normalized=A*scale[:,:,None]*scale[:,None,:]
evals,evecs=np.linalg.eigh(normalized)
post_evals=np.linalg.eigvalsh(Ppost[final])
print('committed covariance min eigenvalue',post_evals.min(),'max asymmetry',np.max(abs(Ppost[final]-Ppost[final].transpose(0,2,1))))
print('NOTE Pcommitted is posterior only on final rows; other rows retain pre-scan covariance.')
rows=[]
for lo,hi in [(3,10),(150,170),(170,180),(180,200),(200,220),(220,240),(240,260),(260,280),(280,300),(300,310),(310,315),(315,320),(320,325),(325,337),(337,360)]:
    m=(d['time']>=lo)&(d['time']<hi)&final
    if not m.any(): continue
    pitch_std=np.sqrt(Ppost[m,1,1])*180/np.pi
    row={'start':lo,'end':hi,'scans':int(m.sum()),'residual_rms_m_median':float(np.median(d['residual_rms'][m])),
      'innovation_rms_m_median':float(np.median(d['innovation_rms'][m])),
      'nis_per_match_median':float(np.median(d['nis'][m]/d['matches'][m])),
      'coupled_normalized_eigenvalues_median':np.median(evals[m],axis=0).tolist(),
      'posterior_rotation_y_std_deg_median':float(np.median(pitch_std)),
      'posterior_ba_std_median':np.median(np.sqrt(np.diagonal(Ppost[m,13:16,13:16],axis1=1,axis2=2)),axis=0).tolist(),
      'posterior_velocity_std_median':np.median(np.sqrt(np.diagonal(Ppost[m,7:10,7:10],axis1=1,axis2=2)),axis=0).tolist(),
      'pitch_bax_corr_median':float(np.median(Ppost[m,1,13]/np.sqrt(Ppost[m,1,1]*Ppost[m,13,13]))),
      'posterior_diag_median':np.median(np.diagonal(Ppost[m],axis1=1,axis2=2),axis=0).tolist(),
      'KH_diag_pose_median':np.median(np.diagonal(KH[m,:6,:],axis1=1,axis2=2),axis=0).tolist(),
      'Kinnovation_sum':Ki[m].sum(axis=0).tolist(),'K0innovation_sum':K0i[m].sum(axis=0).tolist(),
      'noise_std_median':np.median(np.sqrt(noise[m]),axis=0).tolist()}
    rows.append(row)
    print(lo,hi,'RMS',round(row['residual_rms_m_median'],5),'NIS/n',round(row['nis_per_match_median'],3),'normEig',np.round(row['coupled_normalized_eigenvalues_median'],3),'rotY_STDdeg',round(row['posterior_rotation_y_std_deg_median'],5),'baSTD',np.round(row['posterior_ba_std_median'],5),'KHdiag',np.round(row['KH_diag_pose_median'],4),'corr rotY bax',round(row['pitch_bax_corr_median'],3))
    print('  final Kinnov vs same-prior unconstrained: pitch',np.round(np.rad2deg([Ki[m,1].sum(),K0i[m,1].sum()]),4),'velocity',np.round(Ki[m,7:10].sum(axis=0),3),np.round(K0i[m,7:10].sum(axis=0),3),'ba',np.round(Ki[m,13:16].sum(axis=0),3),np.round(K0i[m,13:16].sum(axis=0),3))
a.out.write_text(json.dumps(rows,indent=2))
if a.xicp:
    x=np.genfromtxt(a.xicp,delimiter=',',names=True)
    assert len(x)==len(values) and np.max(abs(x['time']-d['time']))<1e-8
    assert np.array_equal(x['iteration'],d['iteration'])
    all_rot_full=np.all(np.column_stack([x[f'r{k}_category']==2 for k in range(3)]),axis=1)
    results=[]
    for lo,hi in [(170,180),(180,200),(200,220),(180,310)]:
        m=final & (d['time']>=lo) & (d['time']<hi) & all_rot_full
        row={'start':lo,'end':hi,'final_rows_all_rotation_full':int(m.sum()),
             'rotation_y_Kinnov_deg':float(np.rad2deg(Ki[m,1].sum())),
             'rotation_y_K0innov_deg':float(np.rad2deg(K0i[m,1].sum())),
             'ba_x_Kinnov':float(Ki[m,13].sum()),'ba_x_K0innov':float(K0i[m,13].sum())}
        results.append(row); print('ALL_ROTATION_FULL',row)
    a.out.with_name(a.out.stem+'-coupling.json').write_text(json.dumps(results,indent=2))
