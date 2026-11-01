import math
import random
import unittest
from kinematics import ArmKinematics, JointLimitError, UnreachableTarget


class ArmTests(unittest.TestCase):
    def setUp(self): self.arm=ArmKinematics()

    def test_measured_zero(self):
        pose=self.arm.forward((0,0,0))
        self.assertAlmostEqual(pose.x_mm,188.773606084,places=6)
        self.assertAlmostEqual(pose.y_mm,164.847574967,places=6)
        self.assertAlmostEqual(math.degrees(pose.phi_rad),30)
        self.assertEqual(len(pose.points_mm),4)

    def test_motor_direction_and_serial_coupling(self):
        zero=self.arm.forward((0,0,0)).link_angles_rad
        for joint,expected in [(0,(.1,.1,.1)),(1,(0,-.1,-.1)),(2,(0,0,.1))]:
            q=[0,0,0];q[joint]=.1
            pose=self.arm.forward(q)
            for a,b,delta in zip(pose.link_angles_rad,zero,expected):
                self.assertAlmostEqual(a-b,delta)

    def test_roundtrip_10000_positions(self):
        rng=random.Random(2027)
        positions=[(0,0,0),(2.5,5.,5.),(0,math.radians(171),0)]
        positions += [tuple(rng.uniform(lo,hi) for lo,hi in self.arm.limits_rad) for _ in range(10000)]
        for q in positions:
            p=self.arm.forward(q)
            result=self.arm.solve(p.x_mm,p.y_mm,p.phi_rad,seed_rad=q)
            for a,b in zip(q,result): self.assertAlmostEqual(a,b,places=6)

    def test_orientation_periodicity(self):
        q=(.3,1.,2.)
        p=self.arm.forward(q)
        result=self.arm.solve(p.x_mm,p.y_mm,p.phi_rad+4*math.pi,seed_rad=q)
        for a,b in zip(q,result): self.assertAlmostEqual(a,b,places=6)

    def test_target_unreachable(self):
        with self.assertRaises(UnreachableTarget): self.arm.solve(2000,0,0)
        with self.assertRaises(UnreachableTarget): self.arm.solve(1e300,0,0)

    def test_geometric_solution_exceeds_limits(self):
        # A narrow configured interval rejects a geometrically reachable target.
        arm=ArmKinematics(limits_rad=((0,.01),(0,.01),(0,.01)))
        p=self.arm.forward((-1.,.5,.5))
        with self.assertRaises(JointLimitError): arm.solve(p.x_mm,p.y_mm,p.phi_rad)

    def test_expanded_limit_makes_previous_target_reachable(self):
        q=self.arm.solve(800,200,math.radians(30))
        for value,expected in zip(q,(1.042949931,4.439836650,3.396886719)):
            self.assertAlmostEqual(value,expected,places=8)
        p=self.arm.forward(q,check_limits=True)
        self.assertAlmostEqual(p.x_mm,800,places=6)
        self.assertAlmostEqual(p.y_mm,200,places=6)

    def test_multiple_solutions_select_nearest_current_position(self):
        pose=self.arm.forward((1.5,2.,2.))
        candidates=self.arm.inverse(pose.x_mm,pose.y_mm,pose.phi_rad,(1.5,2.,2.))
        self.assertEqual(len(candidates),2)
        for a,b in zip(candidates[0],(1.5,2.,2.)): self.assertAlmostEqual(a,b,places=8)
        selected=self.arm.solve(pose.x_mm,pose.y_mm,pose.phi_rad,candidates[1])
        for a,b in zip(selected,candidates[1]): self.assertAlmostEqual(a,b,places=8)

    def test_id1_limit_filters_old_alternative_branch(self):
        pose=self.arm.forward((1.,1.,1.))
        unrestricted=ArmKinematics(limits_rad=((0,5),(0,5),(0,5)))
        self.assertEqual(len(unrestricted.inverse(pose.x_mm,pose.y_mm,pose.phi_rad)),2)
        candidates=self.arm.inverse(pose.x_mm,pose.y_mm,pose.phi_rad)
        self.assertEqual(len(candidates),1)
        for actual,expected in zip(candidates[0],(1,1,1)):
            self.assertAlmostEqual(actual,expected,places=8)

    def test_equal_link_folded_singularity(self):
        phi=math.radians(30)
        x,y=200*math.cos(phi),200*math.sin(phi)
        # With the corrected zero, fully folding requires ID2=-9 or 351 deg.
        # Neither is inside the actual ID2 interval, so the normal model rejects it.
        with self.assertRaises(JointLimitError): self.arm.solve(x,y,phi)
        relaxed=ArmKinematics(limits_rad=((0,1.57),(0,6.28),(0,6.28)))
        for seed in [(0.,3.,0.),(1.57,3.,1.5)]:
            q=relaxed.solve(x,y,phi,seed)
            p=relaxed.forward(q,check_limits=True)
            self.assertLess(math.hypot(p.x_mm-x,p.y_mm-y),1e-6)
            self.assertAlmostEqual(math.degrees(q[1]),351,places=6)

    def test_old_joint2_limit_is_configurable(self):
        arm=ArmKinematics(limits_rad=((0,1.57),(0,3.14),(0,3.14)))
        q=(.3,1.,2.);p=arm.forward(q)
        result=arm.solve(p.x_mm,p.y_mm,p.phi_rad,q)
        for a,b in zip(q,result): self.assertAlmostEqual(a,b,places=6)

    def test_bad_inputs(self):
        for value in [math.inf,math.nan]:
            with self.assertRaises(ValueError): self.arm.forward((value,0,0))
            with self.assertRaises(ValueError): self.arm.solve(value,0,0)
        with self.assertRaises(ValueError): self.arm.forward((0,0,0,0))

    def test_xy_175_178_frees_orientation_instead_of_violating_limits(self):
        with self.assertRaises(JointLimitError): self.arm.solve(175,178,math.radians(30))
        q=self.arm.solve_position(175,178,(.2,.5,1.))
        pose=self.arm.forward(q,check_limits=True)
        self.assertAlmostEqual(pose.x_mm,175,places=6)
        self.assertAlmostEqual(pose.y_mm,178,places=6)
        self.assertGreater(abs(math.remainder(pose.phi_rad-math.radians(30),2*math.pi)),.1)

    def test_xy_preserves_already_achieved_seed_including_boundaries(self):
        for q in [(0,0,0),(2.5,5,5),(0,5,0),(2.5,0,5),(1.1,.143,4.481)]:
            p=self.arm.forward(q)
            self.assertEqual(self.arm.solve_position(p.x_mm,p.y_mm,q),q)

    def test_xy_random_reachable_targets_with_unrelated_seed(self):
        rng=random.Random(178)
        for i in range(500):
            q=tuple(rng.uniform(lo,hi) for lo,hi in self.arm.limits_rad)
            if i<8: q=tuple(self.arm.limits_rad[j][(i>>j)&1] for j in range(3))
            p=self.arm.forward(q)
            seed=tuple(rng.uniform(lo,hi) for lo,hi in self.arm.limits_rad)
            result=self.arm.solve_position(p.x_mm,p.y_mm,seed)
            actual=self.arm.forward(result,check_limits=True)
            self.assertLess(math.hypot(actual.x_mm-p.x_mm,actual.y_mm-p.y_mm),1e-5)
            # The known feasible q is a useful motion-cost upper bound.
            self.assertLessEqual(sum((v-s)**2 for v,s in zip(result,seed)),
                                 sum((v-s)**2 for v,s in zip(q,seed))+.002)

    def test_xy_narrow_intervals_and_isolated_limit_pose(self):
        q=(.23456789,.87654321,4.7654321)
        p=self.arm.forward(q)
        for width in (1e-7,0.):
            arm=ArmKinematics(limits_rad=tuple((v,v+width) for v in q))
            result=arm.solve_position(p.x_mm,p.y_mm,(0,0,0))
            actual=arm.forward(result,check_limits=True)
            self.assertLess(math.hypot(actual.x_mm-p.x_mm,actual.y_mm-p.y_mm),1e-5)

    def test_xy_rejects_impossible_and_nonfinite_input(self):
        for x,y in [(1051,0),(1e300,0)]:
            with self.assertRaises(UnreachableTarget): self.arm.solve_position(x,y)
        arm=ArmKinematics(limits_rad=((0,0),(0,0),(0,0)))
        with self.assertRaises(JointLimitError): arm.solve_position(175,178)
        for value in (math.nan,math.inf):
            with self.assertRaises(ValueError): self.arm.solve_position(value,178)
            with self.assertRaises(ValueError): self.arm.solve_position(175,178,(0,value,0))


if __name__=='__main__': unittest.main(verbosity=2)
