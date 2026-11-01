"""Planar 3R arm kinematics, measured at the ID4 rotation center.

Distances: mm. Angles: rad. X: horizontal forward, Y: upward.
Motor-zero absolute link angles: -9, +162, +30 degrees.
Motor signs: +ID1 raises link1, +ID2 lowers link2, +ID3 raises link3.
ID4 rotation does not affect the position of its own rotation center.
No serial I/O is performed by this module.
"""
from dataclasses import dataclass
import itertools
import math

TAU = 2 * math.pi
# Matches firmware Lib/arm.h: ID1 0..2.5, ID2/3 0..5 rad.
DEFAULT_LIMITS = ((0., 2.5), (0., 5.), (0., 5.))
EPS = 1e-9


class UnreachableTarget(ValueError):
    """The requested wrist center is outside the two-link geometric workspace."""


class JointLimitError(ValueError):
    """Geometric solutions exist but none satisfies the configured motor limits."""


class CollisionError(ValueError):
    """The modeled links intersect, or a path cannot be certified clear."""


def segment_distance(a, b, c, d):
    def cross(p, q, r):
        return (q[0]-p[0])*(r[1]-p[1])-(q[1]-p[1])*(r[0]-p[0])
    def point_segment(p, u, v):
        dx,dy=v[0]-u[0],v[1]-u[1]
        length=dx*dx+dy*dy
        t=0 if length==0 else min(1,max(0,((p[0]-u[0])*dx+(p[1]-u[1])*dy)/length))
        return math.hypot(p[0]-u[0]-t*dx,p[1]-u[1]-t*dy)
    ab1,ab2=cross(a,b,c),cross(a,b,d)
    cd1,cd2=cross(c,d,a),cross(c,d,b)
    if ((ab1>0 and ab2<0) or (ab1<0 and ab2>0)) and ((cd1>0 and cd2<0) or (cd1<0 and cd2>0)):
        return 0.
    return min(point_segment(a,c,d),point_segment(b,c,d),point_segment(c,a,b),point_segment(d,a,b))


@dataclass(frozen=True)
class CollisionModel:
    # Link1/3 required CENTERLINE separation; use half-width1 + half-width3 + gap.
    # No physical dimensions supplied: zero means centerline intersection only.
    clearance_mm: float = 0.
    fold_margin_rad: float = 0.

    def __post_init__(self):
        if not math.isfinite(self.clearance_mm) or self.clearance_mm<0:
            raise ValueError('中心距必须是非负有限数字')
        if not math.isfinite(self.fold_margin_rad) or not 0<=self.fold_margin_rad<math.pi:
            raise ValueError('折叠禁区必须在 0～180 度之间')

    def _fold_clear(self, arm, start, end):
        a1,a2,a3=arm.zero_angles_rad
        for left,right in [(a2-a1-start[1],a2-a1-end[1]),(a3-a2+start[2],a3-a2+end[2])]:
            lo,hi=sorted((left,right))
            margin=self.fold_margin_rad+1e-7
            if math.ceil((lo-margin-math.pi)/TAU)<=math.floor((hi+margin-math.pi)/TAU):
                return False
        return True

    def clearance(self, arm, joints):
        points=arm.forward(joints,check_limits=True).points_mm
        return segment_distance(*points)-self.clearance_mm

    def check_pose(self, arm, joints):
        joints=finite_tuple(joints,3)
        if not self._fold_clear(arm,joints,joints) or self.clearance(arm,joints)<=1e-5:
            raise CollisionError('连杆发生相交、重叠或进入配置的防干涉间隙')

    def check_path(self, arm, start, end):
        """Certify the whole synchronous straight line in joint space.

        Adaptive distance bounds cover intervals between samples. Ambiguous
        near-contact intervals are rejected rather than assumed safe.
        """
        start,end=finite_tuple(start,3),finite_tuple(end,3)
        self.check_pose(arm,start);self.check_pose(arm,end)
        if not self._fold_clear(arm,start,end):
            raise CollisionError('关节路径穿过相邻连杆反向折叠区域')
        budget=[2048]
        l1,l2,l3=arm.lengths_mm
        def visit(a,b,ca,cb,depth):
            budget[0]-=1
            if budget[0]<0:
                raise CollisionError('路径过于接近干涉边界，请分段调整目标')
            # Bound movement of any point on either segment over this interval.
            bound=(l1+l2+l3)*abs(a[0]-b[0])+(l2+l3)*abs(a[1]-b[1])+l3*abs(a[2]-b[2])
            if min(ca,cb)>2*bound+1e-5:
                return
            if depth==0:
                raise CollisionError('无法确认路径间隙，请远离连杆干涉边界')
            mid=tuple((x+y)/2 for x,y in zip(a,b))
            cm=self.clearance(arm,mid)
            if cm<=1e-5:
                raise CollisionError('目标可达，但途中连杆相交或间隙不足')
            visit(a,mid,ca,cm,depth-1);visit(mid,b,cm,cb,depth-1)
        visit(start,end,self.clearance(arm,start),self.clearance(arm,end),16)


def finite_tuple(values, length):
    result = tuple(float(value) for value in values)
    if len(result) != length or not all(math.isfinite(value) for value in result):
        raise ValueError(f'Expected {length} finite values')
    return result


def periodic_values(angle, limits):
    """All equivalent angles in the actual motor interval; never clamp an IK miss."""
    low, high = limits
    first = math.ceil((low - angle - EPS) / TAU)
    last = math.floor((high - angle + EPS) / TAU)
    return tuple(min(high, max(low, angle + turn * TAU))
                 for turn in range(first, last + 1))


@dataclass(frozen=True)
class Pose:
    x_mm: float
    y_mm: float
    phi_rad: float
    points_mm: tuple
    link_angles_rad: tuple


@dataclass(frozen=True)
class ArmKinematics:
    lengths_mm: tuple = (425., 425., 200.)
    zero_angles_rad: tuple = tuple(math.radians(v) for v in (-9., 162., 30.))
    limits_rad: tuple = DEFAULT_LIMITS

    def __post_init__(self):
        lengths = finite_tuple(self.lengths_mm, 3)
        finite_tuple(self.zero_angles_rad, 3)
        if any(length <= 0 for length in lengths):
            raise ValueError('Link lengths must be positive')
        if len(self.limits_rad) != 3:
            raise ValueError('Expected three pitch motor limits')
        for limits in self.limits_rad:
            lo, hi = finite_tuple(limits, 2)
            if lo > hi:
                raise ValueError('Invalid joint interval')

    def forward(self, joints_rad, check_limits=False):
        q1, q2, q3 = finite_tuple(joints_rad, 3)
        if check_limits:
            for q, (low, high) in zip((q1, q2, q3), self.limits_rad):
                if not low - EPS <= q <= high + EPS:
                    raise JointLimitError('Motor position is outside its limits')
        a1, a2, a3 = self.zero_angles_rad
        angles = (a1 + q1, a2 + q1 - q2, a3 + q1 - q2 + q3)
        points = [(0., 0.)]
        x = y = 0.
        for length, angle in zip(self.lengths_mm, angles):
            x += length * math.cos(angle)
            y += length * math.sin(angle)
            points.append((x, y))
        return Pose(x, y, angles[2], tuple(points), angles)

    def inverse(self, x_mm, y_mm, phi_rad, seed_rad=None):
        """All discrete in-limit IK solutions, nearest the seed first.

        phi is the absolute orientation of link3, not motor ID3 and not ID4.
        For the fully folded equal-link singularity there are infinitely many
        solutions; return a nearest-seed representative of each valid family.
        Raises UnreachableTarget or JointLimitError instead of clipping a target.
        """
        x, y, phi = finite_tuple((x_mm, y_mm, phi_rad), 3)
        phi = math.remainder(phi, TAU)
        seed = finite_tuple(seed_rad if seed_rad is not None else (0., 0., 0.), 3)
        l1, l2, l3 = self.lengths_mm
        a1, a2, a3 = self.zero_angles_rad
        wx, wy = x - l3 * math.cos(phi), y - l3 * math.sin(phi)
        radius = math.hypot(wx, wy)
        if radius > l1 + l2 + 1e-7 or radius < abs(l1 - l2) - 1e-7:
            raise UnreachableTarget(f'Wrist radius {radius:.3f} mm is unreachable')
        cosine = (radius * radius - l1*l1 - l2*l2) / (2*l1*l2)
        elbow = math.acos(min(1., max(-1., cosine)))
        candidates = []
        for gamma in (-elbow, elbow):
            q2_values = periodic_values(a2 - a1 - gamma, self.limits_rad[1])
            if radius < 1e-7 and abs(l1 - l2) < 1e-7:
                # theta1 is free. q3 = phi-a3+q2-q1 + 2*pi*k.
                for q2 in q2_values:
                    base = phi - a3 + q2
                    lo1, hi1 = self.limits_rad[0]
                    lo3, hi3 = self.limits_rad[2]
                    first = math.ceil((lo1 + lo3 - base - EPS) / TAU)
                    last = math.floor((hi1 + hi3 - base + EPS) / TAU)
                    for turn in range(first, last + 1):
                        total = base + TAU * turn
                        lower, upper = max(lo1, total-hi3), min(hi1, total-lo3)
                        if lower > upper + EPS:
                            continue
                        q1 = min(upper, max(lower, (seed[0] + total - seed[2]) / 2))
                        candidates.append((q1, q2, total-q1))
                continue
            theta1 = math.atan2(wy, wx) - math.atan2(l2*math.sin(gamma), l1+l2*math.cos(gamma))
            for q1, q2 in itertools.product(periodic_values(theta1-a1, self.limits_rad[0]), q2_values):
                for q3 in periodic_values(phi-a3-q1+q2, self.limits_rad[2]):
                    candidates.append((q1, q2, q3))

        unique = []
        for candidate in candidates:
            pose = self.forward(candidate, check_limits=True)
            if math.hypot(pose.x_mm-x, pose.y_mm-y) > 1e-5:
                continue
            if abs(math.remainder(pose.phi_rad-phi, TAU)) > 1e-8:
                continue
            if not any(max(abs(a-b) for a,b in zip(candidate,other)) < 1e-8 for other in unique):
                unique.append(candidate)
        if not unique:
            raise JointLimitError('Target requires a motor angle outside the configured limits')
        return sorted(unique, key=lambda q: sum((v-s)**2 for v,s in zip(q,seed)))

    def solve(self, x_mm, y_mm, phi_rad, seed_rad=None):
        """Select the valid solution closest to the current motor feedback."""
        return self.inverse(x_mm, y_mm, phi_rad, seed_rad)[0]

    def solve_position(self, x_mm, y_mm, seed_rad=None):
        """Solve XY with link3 orientation free, preferring little joint motion.

        Use q1 as the redundant coordinate. Analytic roots of every joint-limit
        boundary and two-link tangency split its range into feasible intervals.
        Testing their interiors AND endpoints finds even very narrow intervals
        and isolated boundary poses; this is not a fixed-step orientation scan.
        A sampled one-dimensional refinement favors small squared joint travel
        without claiming a globally optimal motion plan. No collision filtering.
        """
        x, y = finite_tuple((x_mm, y_mm), 2)
        seed = finite_tuple(seed_rad if seed_rad is not None else (0., 0., 0.), 3)
        l1, l2, l3 = self.lengths_mm
        a1, a2, a3 = self.zero_angles_rad
        if math.hypot(x, y) > l1+l2+l3+1e-7:
            raise UnreachableTarget('目标超出机械臂几何工作范围')

        # Keep an already achieved position exactly, including limit boundaries.
        if all(lo <= q <= hi for q, (lo, hi) in zip(seed, self.limits_rad)):
            p = self.forward(seed)
            if math.hypot(p.x_mm-x, p.y_mm-y) < 1e-7:
                return seed

        lo1, hi1 = self.limits_rad[0]
        cuts = [lo1, hi1]
        target_angle, target_radius = math.atan2(y, x), math.hypot(x, y)

        def add_roots(ax, ay, radius):
            # |target - rotate(q1)*A| = radius, solved analytically for q1.
            ar = math.hypot(ax, ay)
            if target_radius < 1e-12 or ar < 1e-12:
                return  # Distance is constant in q1; no crossing to partition.
            cosine = (target_radius**2+ar**2-radius**2)/(2*target_radius*ar)
            if not -1.-1e-12 <= cosine <= 1.+1e-12:
                return
            delta = math.acos(min(1., max(-1., cosine)))
            center = target_angle-math.atan2(ay, ax)
            for angle in (center-delta, center+delta):
                cuts.extend(periodic_values(angle, (lo1, hi1)))

        first_vector = (l1*math.cos(a1), l1*math.sin(a1))
        # Geometric tangencies for the remaining two links.
        for radius in (abs(l2-l3), l2+l3):
            add_roots(*first_vector, radius)
        # q3-limit crossings: the last two links have fixed resultant length.
        for q3 in self.limits_rad[2]:
            radius = math.hypot(l2*math.cos(a2)+l3*math.cos(a3+q3),
                                l2*math.sin(a2)+l3*math.sin(a3+q3))
            add_roots(*first_vector, radius)
        # q2-limit crossings: the first two links have a fixed resultant vector.
        for q2 in self.limits_rad[1]:
            add_roots(first_vector[0]+l2*math.cos(a2-q2),
                      first_vector[1]+l2*math.sin(a2-q2), l3)
        cuts.append(min(hi1, max(lo1, seed[0])))
        cuts = sorted(set(cuts))

        def candidates(q1):
            wx, wy = x-l1*math.cos(a1+q1), y-l1*math.sin(a1+q1)
            radius = math.hypot(wx, wy)
            if radius > l2+l3+1e-7 or radius < abs(l2-l3)-1e-7:
                return []
            cosine = (radius**2-l2*l2-l3*l3)/(2*l2*l3)
            elbow = math.acos(min(1., max(-1., cosine)))
            result = []
            for gamma in (-elbow, elbow):
                q3s = periodic_values(a2-a3+gamma, self.limits_rad[2])
                if radius < 1e-7 and abs(l2-l3) < 1e-7:
                    q2s = (min(self.limits_rad[1][1], max(self.limits_rad[1][0], seed[1])),)
                else:
                    theta2 = math.atan2(wy, wx)-math.atan2(l3*math.sin(gamma), l2+l3*math.cos(gamma))
                    q2s = periodic_values(a2+q1-theta2, self.limits_rad[1])
                for q2, q3 in itertools.product(q2s, q3s):
                    q = (q1, q2, q3)
                    p = self.forward(q, check_limits=True)
                    if math.hypot(p.x_mm-x, p.y_mm-y) <= 1e-5:
                        result.append(q)
            return result

        best, best_cost = None, math.inf

        def evaluate(q1):
            nonlocal best, best_cost
            cost = math.inf
            for q in candidates(q1):
                score = sum((v-s)**2 for v, s in zip(q, seed))
                cost = min(cost, score)
                if score < best_cost:
                    best, best_cost = q, score
            return cost

        for q1 in cuts:
            evaluate(q1)
        for left, right in zip(cuts, cuts[1:]):
            if right-left < 1e-12 or not candidates((left+right)/2):
                continue
            samples = [left+(right-left)*i/8 for i in range(9)]
            scores = [evaluate(q1) for q1 in samples]
            # Refine each sampled local minimum, including boundary minima.
            for i, score in enumerate(scores):
                if score > min(scores[max(0, i-1):min(9, i+2)]):
                    continue
                low, high = samples[max(0, i-1)], samples[min(8, i+1)]
                ratio = (math.sqrt(5)-1)/2
                c, d = high-ratio*(high-low), low+ratio*(high-low)
                fc, fd = evaluate(c), evaluate(d)
                for _ in range(40):
                    if fc < fd:
                        high, d, fd = d, c, fc
                        c = high-ratio*(high-low); fc = evaluate(c)
                    else:
                        low, c, fc = c, d, fd
                        d = low+ratio*(high-low); fd = evaluate(d)
        if best is None:
            raise JointLimitError('此 X/Y 在当前电机限幅内没有逆解（已自动选择姿态）')
        return best

    def solve_safe(self, x_mm, y_mm, phi_rad, seed_rad, collision=None):
        # Compatibility entry: control policy now checks geometry and limits only.
        # The collision argument is intentionally ignored under this policy.
        return self.solve(x_mm,y_mm,phi_rad,seed_rad)



if __name__ == '__main__':
    import argparse
    parser = argparse.ArgumentParser(description=__doc__)
    modes = parser.add_mutually_exclusive_group(required=True)
    modes.add_argument('--fk', nargs=3, type=float, metavar=('Q1_RAD','Q2_RAD','Q3_RAD'))
    modes.add_argument('--ik', nargs=3, type=float, metavar=('X_MM','Y_MM','PHI_DEG'))
    args = parser.parse_args()
    arm = ArmKinematics()
    if args.fk is not None:
        pose = arm.forward(args.fk)
        print(f'X={pose.x_mm:.6f} mm, Y={pose.y_mm:.6f} mm, phi={math.degrees(pose.phi_rad):.6f} deg')
    else:
        try:
            x, y, phi_deg = args.ik
            for q in arm.inverse(x, y, math.radians(phi_deg)):
                print('Motor rad:', ', '.join(f'{v:.9f}' for v in q))
                print('Motor deg:', ', '.join(f'{math.degrees(v):.6f}' for v in q))
        except (UnreachableTarget, JointLimitError) as exc:
            parser.exit(2, str(exc)+'\n')
