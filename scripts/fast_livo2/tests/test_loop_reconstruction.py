"""Geometric acceptance and transform conventions for offline reconstruction."""
import importlib.util
from pathlib import Path
import sys
import unittest

sys.path.insert(0,str(Path(__file__).resolve().parents[1]))
HAS_OPEN3D=importlib.util.find_spec('open3d') is not None
if HAS_OPEN3D:
    import numpy as np
    from scipy.spatial.transform import Rotation
    from optimize_reconstruction import align, cloud, quality, transform, valid_loop, consistent_return_loops, optimize_graph, reg


@unittest.skipUnless(HAS_OPEN3D,'Open3D optional reconstruction dependency')
class LoopReconstructionTests(unittest.TestCase):
    def test_room_registration_recovers_measured_offset(self):
        rng=np.random.default_rng(17)
        surfaces=[]
        for axis,offset in [(0,-3),(0,4),(1,-2),(1,3),(2,0),(2,2.7)]:
            p=rng.uniform([-3,-2,0],[4,3,2.7],(1500,3));p[:,axis]=offset
            surfaces.append(p)
        points=np.concatenate(surfaces)
        truth=np.eye(4);truth[:3,:3]=Rotation.from_euler('xyz',[.03,-.02,.08]).as_matrix()
        truth[:3,3]=[.35,-.18,.1]
        target=cloud(points,.1)
        source=cloud(transform(points,np.linalg.inv(truth)),.1)
        result=align(source,target,np.eye(4))
        self.assertLess(np.linalg.norm(result.transformation[:3,3]-truth[:3,3]),.02)
        self.assertLess(Rotation.from_matrix(result.transformation[:3,:3].T@truth[:3,:3]).magnitude(),.01)
        self.assertTrue(valid_loop(quality(source,target,result.transformation),
                                   quality(target,source,np.linalg.inv(result.transformation))))

    def test_parallel_corridor_cannot_assert_loop(self):
        good=dict(overlap=.95,plane_rmse=.01,normal_eigenvalues=[.0,.4,.6])
        self.assertFalse(valid_loop(good,good))
        good['normal_eigenvalues']=[.1,.4,.5]
        reverse=dict(good,overlap=.1)
        self.assertFalse(valid_loop(good,reverse))

    def test_graph_edge_maps_source_into_target(self):
        graph=reg.PoseGraph()
        start=np.eye(4);end=np.eye(4);end[:3,3]=[3.,1.,.4]
        guess=end.copy();guess[:3,3]+=[.5,-.2,.1]
        graph.nodes.append(reg.PoseGraphNode(start));graph.nodes.append(reg.PoseGraphNode(guess))
        # Source frame 1 -> target frame 0 is end, not its inverse.
        graph.edges.append(reg.PoseGraphEdge(1,0,end,np.eye(6)*100,False))
        reg.global_optimization(graph,reg.GlobalOptimizationLevenbergMarquardt(),
            reg.GlobalOptimizationConvergenceCriteria(),reg.GlobalOptimizationOption(reference_node=0))
        self.assertLess(np.linalg.norm(graph.nodes[1].pose-end),1e-5)
        self.assertGreater(np.linalg.norm(graph.nodes[1].pose[:3,3]),3.)

    def test_return_requires_temporal_consensus_and_rejects_room_symmetry(self):
        poses=np.repeat(np.eye(4)[None],6,axis=0)
        records=[]
        for i in (1,2,3):
            t=np.eye(4);t[:3,3]=[.01*i,0,6]
            records.append(dict(source=i,target=0,source_time=float(i*5),accepted=True,
                                global_search=True,transformation=t.tolist()))
        bad=np.eye(4);bad[:3,:3]=Rotation.from_euler('z',30,degrees=True).as_matrix();bad[2,3]=6
        records.append(dict(source=4,target=0,source_time=20.,accepted=True,
                            global_search=True,transformation=bad.tolist()))
        self.assertEqual([r['source'] for r in consistent_return_loops(records,poses)],[1,2,3])
        self.assertEqual(consistent_return_loops(records[:2],poses),[])

    def test_large_rotation_graph_uses_true_angle_error(self):
        graph=reg.PoseGraph()
        first=np.eye(4);truth=np.eye(4)
        truth[:3,:3]=Rotation.from_euler('z',110,degrees=True).as_matrix();truth[:3,3]=[2.,1.,0]
        guess=truth.copy();guess[:3,:3]=np.eye(3)
        graph.nodes.append(reg.PoseGraphNode(first));graph.nodes.append(reg.PoseGraphNode(guess))
        graph.edges.append(reg.PoseGraphEdge(1,0,truth,np.eye(6)*100,False))
        result=optimize_graph(graph,np.array([first,truth]),np.array([0.,10.]),[])
        self.assertGreater(result['status'],0)
        self.assertLess(np.linalg.norm(graph.nodes[1].pose-truth),1e-4)


if __name__=='__main__':
    unittest.main()
