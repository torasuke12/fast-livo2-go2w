#!/usr/bin/env python3
"""Offline scan registration and verified loop constraints for FAST-LIVO2 exports.

Requires numpy, scipy and Open3D 0.19. Frames are in the IMU coordinate system.
No endpoint position/orientation is prescribed. --return-to-start only proposes
geometric registration candidates; failed registrations never become factors.
"""
import argparse
import hashlib
import json
from pathlib import Path

import numpy as np
import open3d as o3d
from scipy.spatial import cKDTree
from scipy.spatial.transform import Rotation
from scipy.optimize import least_squares
from scipy.sparse import lil_matrix

reg = o3d.pipelines.registration


def cloud(points, voxel=.12):
    p = o3d.geometry.PointCloud(o3d.utility.Vector3dVector(np.asarray(points)))
    if voxel:
        p = p.voxel_down_sample(voxel)
    p.estimate_normals(o3d.geometry.KDTreeSearchParamHybrid(radius=.5, max_nn=30))
    p.orient_normals_towards_camera_location()
    return p


def transform(points, pose):
    return np.asarray(points) @ pose[:3, :3].T + pose[:3, 3]


def align(source, target, initial):
    guess = initial.copy()
    for distance, iterations in ((.8, 40), (.3, 30), (.15, 30)):
        result = reg.registration_icp(source, target, distance, guess,
            reg.TransformationEstimationPointToPlane(reg.HuberLoss(.1)),
            reg.ICPConvergenceCriteria(max_iteration=iterations))
        guess = result.transformation
    return result


def quality(source, target, pose, threshold=.15):
    points = transform(source.points, pose)
    distances, ids = cKDTree(np.asarray(target.points)).query(points)
    mask = distances < threshold
    if not mask.any():
        return dict(overlap=0., rmse=1e6, plane_rmse=1e6, normal_eigenvalues=[0., 0., 0.])
    normals = np.asarray(target.normals)[ids[mask]]
    residual = np.einsum('ij,ij->i', points[mask]-np.asarray(target.points)[ids[mask]], normals)
    return dict(overlap=float(mask.mean()), rmse=float(np.sqrt(np.mean(distances[mask]**2))),
                plane_rmse=float(np.sqrt(np.mean(residual**2))),
                normal_eigenvalues=np.linalg.eigvalsh(normals.T@normals/len(normals)).tolist())


def valid_loop(forward, reverse):
    # Occlusion can make a return-room scan a subset of the initial scan.
    # Still require bidirectional support and three independent normal axes.
    return (min(forward['overlap'], reverse['overlap']) >= .15
            and max(forward['overlap'], reverse['overlap']) >= .30
            and max(forward['plane_rmse'], reverse['plane_rmse']) < .06
            and min(forward['normal_eigenvalues'][0], reverse['normal_eigenvalues'][0]) > .025)


def align_return(source,target,source_attitude,target_attitude):
    """Search translation/yaw near the inertial attitude, not room symmetries.

    A rectangular room rotated by 90 degrees can score higher than its correct
    registration when furniture occludes most of the common surfaces. Bound the
    heading search to +/-45 degrees of the LIO attitude and verify over time.
    """
    a,b=source.voxel_down_sample(.25),target.voxel_down_sample(.25)
    source_center=np.median(np.asarray(a.points),axis=0)
    target_center=np.median(np.asarray(b.points),axis=0)
    best=None;best_score=-1.
    for yaw in (-30.,0.,30.):
        rotation=target_attitude.T@Rotation.from_euler('z',yaw,degrees=True).as_matrix()@source_attitude
        for x in (-2.,0.,2.):
            for y in (-2.,0.,2.):
                guess=np.eye(4);guess[:3,:3]=rotation
                guess[:3,3]=target_center-rotation@source_center+target_attitude.T@np.array([x,y,0.])
                coarse=reg.registration_icp(a,b,1.5,guess,
                    reg.TransformationEstimationPointToPlane(reg.HuberLoss(.3)),reg.ICPConvergenceCriteria(max_iteration=50))
                fit=align(source,target,coarse.transformation)
                correction=target_attitude@fit.transformation[:3,:3]@source_attitude.T
                if Rotation.from_matrix(correction).magnitude()>np.deg2rad(45):continue
                f=quality(source,target,fit.transformation);r=quality(target,source,np.linalg.inv(fit.transformation))
                score=np.sqrt(f['overlap']*r['overlap'])
                if score>best_score:best_score=score;best=fit.transformation
    return best


def consistent_return_loops(records, poses):
    """Require the same map correction at three separated acquisition times.

    Several matches to one scan are not independent temporal confirmations.
    False room symmetries form competing clusters and are rejected.
    """
    candidates=[e for e in records if e.get('global_search') and e['accepted']]
    corrections=[poses[e['target']]@np.asarray(e['transformation'])@np.linalg.inv(poses[e['source']]) for e in candidates]
    groups=[]
    for correction in corrections:
        group=[]
        for i,other in enumerate(corrections):
            delta=np.linalg.inv(correction)@other
            if np.linalg.norm(delta[:3,3])<.5 and Rotation.from_matrix(delta[:3,:3]).magnitude()<np.deg2rad(5):
                group.append(i)
        groups.append(group)
    if not groups:return []
    best=max(groups,key=lambda g:len({candidates[i]['source'] for i in g}))
    selected=[candidates[i] for i in best]
    times=[e['source_time'] for e in selected]
    if len(set(e['source'] for e in selected))<3 or max(times)-min(times)<5:
        return []
    support=len(set(e['source'] for e in selected))
    if any(not set(g).intersection(best) and len({candidates[i]['source'] for i in g})>=support for g in groups):
        return []
    return selected


def load_frames(root):
    rows = np.genfromtxt(root/'poses.csv', delimiter=',', names=True)
    poses = np.repeat(np.eye(4)[None], len(rows), axis=0)
    poses[:, :3, :3] = Rotation.from_quat(np.column_stack([rows[n] for n in ('qx','qy','qz','qw')])).as_matrix()
    poses[:, :3, 3] = np.column_stack([rows[n] for n in ('px','py','pz')])
    return rows, poses


def optimize_graph(graph, attitude_reference, times, confirmed, sequential_quality=None):
    """Use SO(3) logarithms: a 180-degree error must never have zero residual.

    Fixed node 0 removes the gauge. Weak roll/pitch priors preserve the IMU's
    gravity reference while leaving yaw and every position free to close loops.
    """
    initial=np.array([n.pose for n in graph.nodes]);count=len(initial)
    if confirmed:
        e=confirmed[0]
        correction=initial[e['target']]@np.asarray(e['transformation'])@np.linalg.inv(initial[e['source']])
        angle=Rotation.from_matrix(correction[:3,:3]).as_rotvec()
        for i,t in enumerate(times):
            fraction=np.clip((t-times[0])/(e['source_time']-times[0]),0.,1.)
            c=np.eye(4);c[:3,:3]=Rotation.from_rotvec(angle*fraction).as_matrix();c[:3,3]=correction[:3,3]*fraction
            initial[i]=c@initial[i]
    source=np.array([e.source_node_id for e in graph.edges]);target=np.array([e.target_node_id for e in graph.edges])
    measurements=np.array([e.transformation for e in graph.edges])
    roots=[]
    for e in graph.edges:
        info=np.asarray(e.information)
        support=max(1.,np.trace(info[3:,3:])/3.)
        # Normalize correlated surface samples, then use a 5 cm registration
        # scale instead of treating every return as an independent pose sensor.
        sigma=.05
        quality_record=(sequential_quality or {}).get((e.source_node_id,e.target_node_id))
        if quality_record:
            q=quality_record['quality']
            sigma=max(.02,q['plane_rmse'])/max(.2,q['overlap'])
            if not quality_record['accepted']:sigma=max(.5,sigma)
        roots.append(np.linalg.cholesky(info/support/sigma**2+np.eye(6)*1e-6).T)
    roots=np.array(roots)
    reference=attitude_reference[:,:3,:3]
    fixed=initial[0].copy()
    def unpack(x):
        values=x.reshape(-1,6)
        rotations=np.concatenate((fixed[None,:3,:3],Rotation.from_rotvec(values[:,:3]).as_matrix()))
        positions=np.concatenate((fixed[None,:3,3],values[:,3:]))
        return rotations,positions
    def residual(x):
        rotations,positions=unpack(x)
        target_inverse=rotations[target].transpose(0,2,1)
        predicted_r=target_inverse@rotations[source]
        error_r=predicted_r@measurements[:,:3,:3].transpose(0,2,1)
        predicted_t=np.einsum('nij,nj->ni',target_inverse,positions[source]-positions[target])
        error_t=predicted_t-np.einsum('nij,nj->ni',error_r,measurements[:,:3,3])
        errors=np.column_stack((Rotation.from_matrix(error_r).as_rotvec(),error_t))
        weighted=np.einsum('nij,nj->ni',roots,errors).ravel()
        # Compare the body-frame direction of world gravity; yaw is unobserved.
        gravity=(rotations[1:].transpose(0,2,1)[:,:,2]-reference[1:].transpose(0,2,1)[:,:,2])/np.deg2rad(5.)
        return np.concatenate((weighted,gravity.ravel()))
    pattern=lil_matrix((6*len(source)+3*(count-1),6*(count-1)),dtype=int)
    for k,(i,j) in enumerate(zip(source,target)):
        for n in (i,j):
            if n:pattern[k*6:k*6+6,(n-1)*6:n*6]=1
    for i in range(1,count):
        pattern[6*len(source)+3*(i-1):6*len(source)+3*i,(i-1)*6:(i-1)*6+3]=1
    x=np.column_stack((Rotation.from_matrix(initial[1:,:3,:3]).as_rotvec(),initial[1:,:3,3])).ravel()
    solution=least_squares(residual,x,jac_sparsity=pattern.tocsr(),x_scale='jac',loss='soft_l1',f_scale=2.5,
                           max_nfev=600,ftol=1e-6,xtol=1e-7,gtol=1e-6,verbose=1)
    if not np.isfinite(solution.x).all():raise RuntimeError('Nonfinite pose graph solution')
    rotations,positions=unpack(solution.x)
    for i,n in enumerate(graph.nodes):
        p=np.eye(4);p[:3,:3]=rotations[i];p[:3,3]=positions[i];n.pose=p
    return dict(cost=float(solution.cost),evaluations=solution.nfev,optimality=float(solution.optimality),status=int(solution.status))


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('capture', type=Path)
    parser.add_argument('output', type=Path)
    parser.add_argument('--submap-seconds', type=float, default=2.5)
    parser.add_argument('--return-to-start', action='store_true')
    parser.add_argument('--no-refine', action='store_true')
    parser.add_argument('--reuse-registrations', type=Path,
                        help='Re-evaluate saved geometric registrations for the identical frontend trajectory')
    args = parser.parse_args()
    if args.output.exists():
        parser.error('output already exists')
    summary = json.loads((args.capture/'summary.json').read_text())
    if not summary.get('complete') or not summary.get('bag_exhausted'):
        parser.error('requires a completed whole-bag capture')
    args.output.mkdir(parents=True)
    o3d.utility.random.seed(7)
    rows, initial_frames = load_frames(args.capture)
    signature=hashlib.sha256((args.capture/'poses.csv').read_bytes())
    cached={}
    if args.reuse_registrations:
        archive=np.load(args.reuse_registrations/'trajectory.npz')
        if not np.array_equal(archive['original'],initial_frames) or not np.array_equal(archive['time'],rows['time']):
            parser.error('registration cache has a different frontend trajectory')
        saved=json.loads((args.reuse_registrations/'registrations.json').read_text())
        cached={(e['source'],e['target'],e['global_search']):e for e in saved
                if e['kind']=='loop' and e.get('phase',1)==1}
    groups = np.floor((rows['time']-rows['time'][0])/args.submap_seconds).astype(int)
    anchors, times, scans, heldout, frame_groups = [], [], [], [], []
    index_groups=[]
    for group in np.unique(groups):
        indices = np.flatnonzero(groups == group)
        rotations=initial_frames[indices,:3,:3]
        angle=Rotation.from_matrix(rotations@rotations[0].T).magnitude().max()
        # A rigid multi-second submap would bake stair/turn pose errors into its
        # geometry. Keep individual acquisitions adjustable in these intervals.
        if np.ptp(initial_frames[indices,2,3])>.3 or angle>np.deg2rad(20):
            index_groups.extend([np.array([i]) for i in indices])
        else:index_groups.append(indices)
    anchor_indices=[]
    for indices in index_groups:
        anchor_indices.append(indices[len(indices)//2])
        anchor = initial_frames[indices[len(indices)//2]]
        train, test = [], []
        for i in indices:
            raw=(args.capture/'scans'/f'{int(rows[i]["id"])}.bin').read_bytes()
            signature.update(raw)
            points = np.frombuffer(raw, dtype=np.float32).reshape(-1,3)
            points = points[np.isfinite(points).all(axis=1) & (np.linalg.norm(points, axis=1)<35)]
            points = transform(points, np.linalg.inv(anchor)@initial_frames[i])
            train.append(points[::2]);test.append(points[1::2])
        scans.append(cloud(np.concatenate(train)))
        heldout.append(cloud(np.concatenate(test)))
        anchors.append(anchor);times.append(float(np.mean(rows['time'][indices])));frame_groups.append(indices)
    original = np.array(anchors)
    identity=dict(sha256=signature.hexdigest(),submap_seconds=args.submap_seconds,local_refine=not args.no_refine,
                  adaptive_submaps=True)
    (args.output/'input_signature.json').write_text(json.dumps(identity,indent=2))
    if args.reuse_registrations:
        cache_signature=args.reuse_registrations/'input_signature.json'
        if not cache_signature.exists() or json.loads(cache_signature.read_text())!=identity:
            parser.error('registration cache is not verified for these exact point clouds and options')
    if args.reuse_registrations and not np.array_equal(archive['submap_original'],original):
        parser.error('registration cache uses different submaps')
    poses = original.copy()
    graph = reg.PoseGraph()
    graph.nodes.append(reg.PoseGraphNode(poses[0]))
    records = []
    # Local registration corrects relative geometric drift before proposing loops.
    for i in range(1,len(scans)):
        if args.reuse_registrations:
            continue
        relative = np.linalg.inv(original[i-1])@original[i]
        dt=rows['time'][anchor_indices[i]]-rows['time'][anchor_indices[i-1]]
        previous=rows[anchor_indices[i-1]]
        velocity=np.array([previous['vx'],previous['vy'],previous['vz']])
        displacement=original[i-1,:3,:3].T@velocity*dt
        predicted=False
        # Temporal smoothing can abruptly revise position without an equivalent
        # physical velocity. Use its inertial velocity only as an ICP initial
        # guess when that discontinuity is large; geometry still decides motion.
        if np.linalg.norm(relative[:3,3]-displacement)>max(.5,3.*np.linalg.norm(displacement)):
            relative[:3,3]=displacement;predicted=True
        local = []
        for j in range(max(0,i-3),i):
            local.append(transform(scans[j].points, np.linalg.inv(poses[i-1])@poses[j]))
        target = cloud(np.concatenate(local))
        result = align(scans[i], target, relative)
        q = quality(scans[i],target,result.transformation)
        correction = np.linalg.inv(relative)@result.transformation
        accepted = (q['overlap']>.35 and q['plane_rmse']<.08
                    and np.linalg.norm(correction[:3,3])<2.
                    and Rotation.from_matrix(correction[:3,:3]).magnitude()<.35)
        if accepted and not args.no_refine:
            relative=result.transformation
        poses[i]=poses[i-1]@relative
        graph.nodes.append(reg.PoseGraphNode(poses[i]))
        info=reg.get_information_matrix_from_point_clouds(scans[i],target,.2,relative)
        graph.edges.append(reg.PoseGraphEdge(i,i-1,relative,info,uncertain=False))
        records.append(dict(kind='sequential',source=i,target=i-1,accepted=bool(accepted),
                            velocity_initialization=predicted,quality=q))
        if i%20==0:
            print(f'sequential {i}/{len(scans)} t={times[i]:.1f} p={poses[i,:3,3]}',flush=True)
    if args.reuse_registrations:
        graph=o3d.io.read_pose_graph(str(args.reuse_registrations/'local_graph.json'))
        poses=np.array([n.pose for n in graph.nodes])
        records=[e for e in saved if e['kind']=='sequential']
    np.save(args.output/'local_poses.npy',poses)
    o3d.io.write_pose_graph(str(args.output/'local_graph.json'),graph)
    (args.output/'registrations.json').write_text(json.dumps(records,indent=2))
    # Spatial candidates plus explicit return-room search. The latter is a search
    # hint, not a constraint that the first and last robot poses are identical.
    candidates=set()
    for i in range(len(scans)):
        distances=np.linalg.norm(poses[:i,:3,3]-poses[i,:3,3],axis=1)
        possible=[j for j in np.argsort(distances) if times[i]-times[j]>30 and distances[j]<12]
        for j in possible[:3]:
            candidates.add((i,int(j),False))
    if args.return_to_start:
        first=[i for i,t in enumerate(times) if t-times[0]<15]
        last=[i for i,t in enumerate(times) if times[-1]-t<20]
        for i in last[::2]:
            for j in first[::2]:
                candidates.add((i,j,True))
    accepted_loops=[]
    return_seed=None
    searched_sources=set()
    for k,(i,j,global_search) in enumerate(sorted(candidates)):
        relative=np.linalg.inv(poses[j])@poses[i]
        cache=cached.get((i,j,global_search))
        if global_search:
            if i not in searched_sources or return_seed is None:
                pose=align_return(scans[i],scans[j],original[i,:3,:3],original[j,:3,:3])
                searched_sources.add(i)
                if pose is None:continue
                return_seed=poses[j]@pose@np.linalg.inv(poses[i])
            else:
                pose=align(scans[i],scans[j],np.linalg.inv(poses[j])@return_seed@poses[i]).transformation
        else:
            pose=(np.array(cache['transformation']) if cache is not None
                  else align(scans[i],scans[j],relative).transformation)
        forward=quality(heldout[i],heldout[j],pose)
        reverse=quality(heldout[j],heldout[i],np.linalg.inv(pose))
        accepted=valid_loop(forward,reverse)
        world_correction=poses[j]@pose@np.linalg.inv(poses[i])
        gravity_angle=float(np.arccos(np.clip(world_correction[2,2],-1.,1.)))
        accepted=bool(accepted and gravity_angle<np.deg2rad(25.))
        if global_search:
            inertial_difference=original[j,:3,:3]@pose[:3,:3]@original[i,:3,:3].T
            accepted=accepted and Rotation.from_matrix(inertial_difference).magnitude()<np.deg2rad(45.)
            accepted=bool(accepted)
        vertical_correction=float((poses[j]@pose)[2,3]-poses[i,2,3])
        # Repeated floor plans can align extremely well geometrically. Ordinary
        # spatial candidates cannot move a scan onto another floor. The explicit
        # return-room search is allowed to repair accumulated vertical drift.
        if (not global_search and abs(vertical_correction)>1.0
                and abs(original[i,2,3]-original[j,2,3])>1.5):
            accepted=False
        record=dict(kind='loop',source=i,target=j,source_time=times[i],target_time=times[j],
                    global_search=global_search,accepted=accepted,forward=forward,reverse=reverse,
                    transformation=pose.tolist(),gravity_angle_degrees=float(np.rad2deg(gravity_angle)),
                    vertical_correction=vertical_correction)
        records.append(record)
        if accepted and not global_search:
            info=reg.get_information_matrix_from_point_clouds(scans[i],scans[j],.15,pose)
            graph.edges.append(reg.PoseGraphEdge(i,j,pose,info,uncertain=True))
            accepted_loops.append(record)
            print(f'loop {i}->{j} t={times[i]:.1f}/{times[j]:.1f} overlap={forward["overlap"]:.2f}/{reverse["overlap"]:.2f}',flush=True)
        if k%50==0:
            print(f'loop candidates {k}/{len(candidates)}',flush=True)
    confirmed=consistent_return_loops(records,poses)
    for e in records:
        if e.get('global_search'):
            e['confirmed']=any(e is c for c in confirmed)
    for e in confirmed:
        i,j=e['source'],e['target'];pose=np.asarray(e['transformation'])
        info=reg.get_information_matrix_from_point_clouds(scans[i],scans[j],.15,pose)
        # Data association is independently confirmed by multiple return times.
        # Keep these measured constraints even when initial drift is large.
        graph.edges.append(reg.PoseGraphEdge(i,j,pose,info,uncertain=False))
        accepted_loops.append(e)
    print(f'confirmed return constraints: {len(confirmed)}',flush=True)
    (args.output/'registrations.json').write_text(json.dumps(records,indent=2))
    o3d.io.write_pose_graph(str(args.output/'graph_before.json'),graph)
    sequential_quality={(e['source'],e['target']):e for e in records if e['kind']=='sequential'}
    optimization=optimize_graph(graph,original,np.array(times),confirmed,sequential_quality)
    # Closing the room makes additional same-floor revisits searchable. In
    # particular, the two upper-corridor traversals can be many metres apart
    # before the first graph update. Keep floor compatibility from the frontend
    # as well as the current registration correction, to avoid floor aliases.
    if confirmed:
        current=np.array([n.pose for n in graph.nodes])
        known={(e.source_node_id,e.target_node_id) for e in graph.edges}
        added=0
        for i in range(len(scans)):
            distance=np.linalg.norm(current[:i,:3,3]-current[i,:3,3],axis=1)
            nearby=[int(j) for j in np.argsort(distance)
                    if times[i]-times[j]>30 and distance[j]<12 and (i,int(j)) not in known][:3]
            for j in nearby:
                pose=align(scans[i],scans[j],np.linalg.inv(current[j])@current[i]).transformation
                f=quality(heldout[i],heldout[j],pose);r=quality(heldout[j],heldout[i],np.linalg.inv(pose))
                correction=current[j]@pose@np.linalg.inv(current[i])
                dz=float((current[j]@pose)[2,3]-current[i,2,3])
                accepted=(valid_loop(f,r) and min(f['overlap'],r['overlap'])>.2
                          and max(f['overlap'],r['overlap'])>.4
                          and Rotation.from_matrix(correction[:3,:3]).magnitude()<np.deg2rad(30)
                          and (abs(dz)<1. or abs(original[i,2,3]-original[j,2,3])<1.5))
                record=dict(kind='loop',phase=2,source=i,target=j,source_time=times[i],target_time=times[j],
                            global_search=False,accepted=bool(accepted),forward=f,reverse=r,transformation=pose.tolist())
                records.append(record)
                if accepted:
                    info=reg.get_information_matrix_from_point_clouds(scans[i],scans[j],.15,pose)
                    graph.edges.append(reg.PoseGraphEdge(i,j,pose,info,uncertain=True))
                    accepted_loops.append(record);added+=1
            if i%50==0:print(f'second loop pass {i}/{len(scans)}, added={added}',flush=True)
        if added:
            optimization=dict(first=optimization,second=optimize_graph(graph,original,np.array(times),confirmed,sequential_quality))
    (args.output/'registrations.json').write_text(json.dumps(records,indent=2))
    o3d.io.write_pose_graph(str(args.output/'graph_after.json'),graph)
    optimized=np.array([n.pose for n in graph.nodes])
    corrected=initial_frames.copy()
    all_points=[]
    for i,indices in enumerate(frame_groups):
        correction=optimized[i]@np.linalg.inv(original[i])
        corrected[indices]=correction@initial_frames[indices]
        all_points.append(transform(scans[i].points,optimized[i]))
        all_points.append(transform(heldout[i].points,optimized[i]))
    combined=cloud(np.concatenate(all_points),.08)
    o3d.io.write_point_cloud(str(args.output/'map.ply'),combined)
    q=Rotation.from_matrix(corrected[:,:3,:3]).as_quat()
    np.savetxt(args.output/'trajectory.tum',np.column_stack((rows['stamp'],corrected[:,:3,3],q)),fmt='%.9f')
    np.savez_compressed(args.output/'trajectory.npz',time=rows['time'],original=initial_frames,corrected=corrected,
                        submap_times=times,submap_original=original,submap_local=poses,submap_corrected=optimized)
    result=dict(submaps=len(scans),keyframes=len(rows),accepted_loops=len(accepted_loops),
                complete=not args.return_to_start or bool(confirmed),
                confirmed_return_constraints=len(confirmed),
                optimization=optimization,
                retained_loops=sum(e.uncertain for e in graph.edges)+len(confirmed),map_points=len(combined.points),
                endpoint_displacement_before=float(np.linalg.norm(initial_frames[-1,:3,3]-initial_frames[0,:3,3])),
                endpoint_displacement_after=float(np.linalg.norm(corrected[-1,:3,3]-corrected[0,:3,3])),
                note='Endpoint displacement is not absolute trajectory error; the robot need not return to the identical pose.')
    (args.output/'summary.json').write_text(json.dumps(result,indent=2))
    print(json.dumps(result,indent=2),flush=True)
    if not result['complete']:
        raise SystemExit('No temporally confirmed return-room registration; output is diagnostic, not a verified closed reconstruction.')


if __name__=='__main__':
    main()
