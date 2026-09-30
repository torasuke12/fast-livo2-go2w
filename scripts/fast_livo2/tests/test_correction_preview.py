"""Safety and frame conventions at the live/offline visualization boundary."""
import json
from pathlib import Path
import sys
import tempfile
import unittest

import numpy as np
from scipy.spatial.transform import Rotation

sys.path.insert(0, str(Path(__file__).resolve().parents[1]))
from correction_preview import KeyframePreview, validate_correction


class CorrectionPreviewTests(unittest.TestCase):
    def setUp(self):
        self.temp = tempfile.TemporaryDirectory()
        self.root = Path(self.temp.name)
        (self.root/'scans').mkdir()
        self.csv = self.root/'poses.csv'
        self.csv.write_text('id,time,stamp,px,py,pz,qx,qy,qz,qw,vx,vy,vz,bgx,bgy,bgz,bax,bay,baz,points\n')
        self.reader = KeyframePreview(self.root)

    def tearDown(self):
        self.temp.cleanup()

    def frame(self):
        q = Rotation.from_euler('z', 90, degrees=True).as_quat()
        row = [0, 4., 1234., 3., 4., 5., *q, *([0]*9), 1]
        np.array([[1, 0, 0]], dtype='<f4').tofile(self.root/'scans/0.bin')
        return ','.join(map(str, row))+'\n'

    def test_partial_csv_is_not_a_committed_frame(self):
        line = self.frame()
        with self.csv.open('a') as f:
            f.write(line[:-5])
        self.assertEqual(len(self.reader.update()), 0)
        with self.csv.open('a') as f:
            f.write(line[-5:])
        np.testing.assert_allclose(self.reader.update(), [[3, 5, 5]], atol=1e-10)
        self.assertEqual(len(self.reader.update()), 0)
        np.testing.assert_allclose(self.reader.trajectory[0, :4], [1234, 3, 4, 5])

    def test_truncated_scan_cannot_be_shown_as_complete(self):
        self.csv.write_text(self.csv.read_text()+self.frame())
        (self.root/'scans/0.bin').write_bytes(b'')
        with self.assertRaisesRegex(ValueError, 'Incomplete committed scan'):
            self.reader.update()

    def correction(self):
        self.csv.write_text(self.csv.read_text()+self.frame())
        self.reader.update()
        out = self.root/'optimized'
        out.mkdir()
        (out/'summary.json').write_text(json.dumps(dict(complete=True)))
        np.savetxt(out/'trajectory.tum', self.reader.trajectory)
        poses = np.eye(4)[None]
        poses[0, :3, 3] = [3, 4, 5]
        poses[0, :3, :3] = Rotation.from_euler('z', 90, degrees=True).as_matrix()
        np.savez(out/'trajectory.npz', original=poses)
        return out

    def test_rejects_backend_from_a_different_run(self):
        out = self.correction()
        validate_correction(self.reader, out)
        np.savez(out/'trajectory.npz', original=np.eye(4)[None])
        with self.assertRaisesRegex(ValueError, 'source poses differ'):
            validate_correction(self.reader, out)

    def test_unverified_return_is_never_applied(self):
        out = self.correction()
        (out/'summary.json').write_text(json.dumps(dict(complete=False)))
        with self.assertRaisesRegex(ValueError, 'did not verify'):
            validate_correction(self.reader, out)


if __name__ == '__main__':
    unittest.main()
