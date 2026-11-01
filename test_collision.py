import math
import random
import unittest
from kinematics import ArmKinematics, CollisionModel, CollisionError, segment_distance

class CollisionTests(unittest.TestCase):
    def setUp(self): self.arm=ArmKinematics();self.model=CollisionModel()

    def test_segment_intersection_touch_overlap_and_separation(self):
        self.assertEqual(segment_distance((0,0),(10,10),(0,10),(10,0)),0)
        self.assertEqual(segment_distance((0,0),(10,0),(5,0),(20,0)),0)
        self.assertEqual(segment_distance((0,0),(10,0),(10,0),(20,0)),0)
        self.assertAlmostEqual(segment_distance((0,0),(10,0),(0,3),(10,3)),3)

    def test_zero_and_known_collision(self):
        self.model.check_pose(self.arm,(0,0,0)) # Shared joint positions allowed.
        with self.assertRaises(CollisionError):
            self.model.check_pose(self.arm,(1.10190555,.14316013,4.48090835))

    def test_safe_ik_and_forward_residual(self):
        q=self.arm.solve_safe(800,200,math.pi/6,(0,0,0))
        self.model.check_path(self.arm,(0,0,0),q)
        p=self.arm.forward(q)
        self.assertLess(math.hypot(p.x_mm-800,p.y_mm-200),1e-6)

    def test_legacy_solve_safe_now_accepts_overlap(self):
        p=self.arm.forward((1.10190555,.14316013,4.48090835))
        q=self.arm.solve_safe(p.x_mm,p.y_mm,p.phi_rad,(0,0,0))
        actual=self.arm.forward(q,check_limits=True)
        self.assertLess(math.hypot(actual.x_mm-p.x_mm,actual.y_mm-p.y_mm),1e-6)
        with self.assertRaises(CollisionError):self.model.check_pose(self.arm,q)

    def test_adjacent_fold_between_clear_endpoints(self):
        arm=ArmKinematics(limits_rad=((0,2.5),(0,5),(0,6)))
        a,b=(1,3,2),(1,3,6)
        self.model.check_pose(arm,a);self.model.check_pose(arm,b)
        with self.assertRaises(CollisionError): self.model.check_path(arm,a,b)
        with self.assertRaises(CollisionError): self.model.check_pose(arm,(1,3,math.radians(312)))

    def test_certified_paths_against_dense_sampling(self):
        rng=random.Random(67)
        accepted=0
        for _ in range(150):
            a=tuple(rng.uniform(lo,hi) for lo,hi in self.arm.limits_rad)
            b=tuple(rng.uniform(lo,hi) for lo,hi in self.arm.limits_rad)
            try:self.model.check_path(self.arm,a,b)
            except CollisionError:continue
            accepted+=1
            for i in range(301):
                q=tuple(x+(y-x)*i/300 for x,y in zip(a,b))
                self.model.check_pose(self.arm,q)
        self.assertGreater(accepted,50)

if __name__=='__main__':unittest.main(verbosity=2)
