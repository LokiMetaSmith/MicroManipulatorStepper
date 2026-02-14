# FreeCAD Testing & Simulation - Kinematic Model
# Port of firmware/MotionControllerRP/src/kinematic_models/kinematic_model_delta3d.cpp

import math
import logging

logging.basicConfig(level=logging.INFO)
logger = logging.getLogger(__name__)

# Constants
DEG2RAD = math.pi / 180.0
RAD2DEG = 180.0 / math.pi

class Vec3F:
    def __init__(self, x=0.0, y=0.0, z=0.0):
        self.x = float(x)
        self.y = float(y)
        self.z = float(z)

    def __add__(self, other):
        return Vec3F(self.x + other.x, self.y + other.y, self.z + other.z)

    def __sub__(self, other):
        return Vec3F(self.x - other.x, self.y - other.y, self.z - other.z)

    def __mul__(self, scalar):
        return Vec3F(self.x * scalar, self.y * scalar, self.z * scalar)

    def __truediv__(self, scalar):
        t = 1.0 / scalar
        return Vec3F(self.x * t, self.y * t, self.z * t)

    def __repr__(self):
        return f"Vec3F({self.x:.4f}, {self.y:.4f}, {self.z:.4f})"

    def dot(self, other):
        return self.x * other.x + self.y * other.y + self.z * other.z

    def cross(self, other):
        return Vec3F(
            self.y * other.z - self.z * other.y,
            self.z * other.x - self.x * other.z,
            self.x * other.y - self.y * other.x
        )

    def length(self):
        return math.sqrt(self.x**2 + self.y**2 + self.z**2)

    def sqr_length(self):
        return self.x**2 + self.y**2 + self.z**2

    def normalized(self):
        length = self.length()
        if length > 0.0:
            return self / length
        return Vec3F(0.0, 0.0, 0.0)

class QuaternionF:
    def __init__(self, w=1.0, x=0.0, y=0.0, z=0.0):
        self.w = float(w)
        self.x = float(x)
        self.y = float(y)
        self.z = float(z)

    @staticmethod
    def from_axis_angle(axis, angle_rad):
        naxis = axis.normalized()
        half_angle = 0.5 * angle_rad
        s = math.sin(half_angle)
        return QuaternionF(math.cos(half_angle), naxis.x * s, naxis.y * s, naxis.z * s)

    def normalized(self):
        norm = math.sqrt(self.w**2 + self.x**2 + self.y**2 + self.z**2)
        if norm > 0.0:
            return QuaternionF(self.w / norm, self.x / norm, self.y / norm, self.z / norm)
        return QuaternionF()

    def normalized_inverse(self):
        return QuaternionF(self.w, -self.x, -self.y, -self.z)

    def rotate(self, v):
        qvec = Vec3F(self.x, self.y, self.z)
        t = qvec.cross(v) * 2.0
        return v + t * self.w + qvec.cross(t)

    def __mul__(self, other):
        return QuaternionF(
            self.w * other.w - self.x * other.x - self.y * other.y - self.z * other.z,
            self.w * other.x + self.x * other.w + self.y * other.z - self.z * other.y,
            self.w * other.y - self.x * other.z + self.y * other.w + self.z * other.x,
            self.w * other.z + self.x * other.y - self.y * other.x + self.z * other.w
        )

class Pose6DF:
    def __init__(self, translation=None, rotation=None):
        self.translation = translation if translation else Vec3F()
        self.rotation = rotation.normalized() if rotation else QuaternionF()

    def transformPoint(self, localPoint):
        return self.rotation.rotate(localPoint) + self.translation

    def inverse(self):
        inv_rot = self.rotation.normalized_inverse()
        inv_trans = inv_rot.rotate(self.translation * -1.0)
        return Pose6DF(inv_trans, inv_rot)

def circle_sphere_intersection(r1, p, r2):
    """
    Computes intersections of a circle in XY-plane (radius r1) and a sphere (center p, radius r2).
    Returns list of Vec3F intersections.
    """
    px, py, pz = p.x, p.y, p.z
    r2_sq = r2**2
    pz_sq = pz**2

    r_proj_sq = r2_sq - pz_sq
    if r_proj_sq < 0.0:
        return []

    r_proj = math.sqrt(r_proj_sq)
    d_sq = px**2 + py**2
    sum_r = r1 + r_proj
    diff_r = abs(r1 - r_proj)

    if d_sq > sum_r**2 or d_sq < diff_r**2:
        return []

    d = math.sqrt(d_sq)
    if d < 1e-9: # Concentric circles
        return [] # Infinite solutions or none depending on radii, here assume none for unique joints

    inv_d = 1.0 / d
    r1_sq = r1**2
    a = (r1_sq - r_proj_sq + d_sq) * 0.5 * inv_d

    h_sq = r1_sq - a**2
    if h_sq < 0.0:
        return [] # Numerical issue
    h = math.sqrt(h_sq)

    cx2 = px * (a * inv_d)
    cy2 = py * (a * inv_d)
    rx = -py * (h * inv_d)
    ry = px * (h * inv_d)

    return [
        Vec3F(cx2 + rx, cy2 + ry, 0.0),
        Vec3F(cx2 - rx, cy2 - ry, 0.0)
    ]

def three_sphere_intersection(p1, r1, p2, r2, p3, r3):
    """
    Computes intersection of three spheres. Returns list of Vec3F points.
    """
    eps = 1e-7
    r1_sqr = r1**2

    ex = p2 - p1
    d2 = ex.sqr_length()
    if d2 < eps: return []

    d = math.sqrt(d2)
    inv_d = 1.0 / d
    ex = ex * inv_d

    temp = p3 - p1
    i = ex.dot(temp)

    ey = temp - ex * i
    ey2 = ey.sqr_length()
    if ey2 < eps: return []

    inv_ey = 1.0 / math.sqrt(ey2)
    ey = ey * inv_ey
    j = ey.dot(temp)

    ez = ex.cross(ey)

    x = (r1_sqr - r2**2 + d**2) * 0.5 * inv_d
    y = (r1_sqr - r3**2 + i**2 + j**2 - 2.0 * i * x) * 0.5 * inv_ey

    z2 = r1_sqr - x**2 - y**2
    if z2 < eps: return []

    z = math.sqrt(z2)
    base = p1 + ex * x + ey * y

    return [base + ez * z, base - ez * z]


class KinematicModel_Delta3D:
    def __init__(self):
        # Parameters from C++ code
        base_offset = Vec3F(-32.5, -32.5, -32.5)
        self.arm_length = 73.8
        self.rotor_radius = 15.0
        self.ee_attachment_points = [
            Vec3F(-0.5, -14.5, 2.0),
            Vec3F(2.0, -0.5, -14.5),
            Vec3F(-14.5, 2.0, -0.5)
        ]

        self.actuator_to_base = [Pose6DF() for _ in range(3)]
        self.base_to_actuator = [Pose6DF() for _ in range(3)]
        self.rotor_angle_offset = [0.0] * 3

        # Setup transformations
        self.actuator_to_base[0].rotation = QuaternionF.from_axis_angle(Vec3F(0.0, 0.0, 1.0), 90.0 * DEG2RAD)
        self.actuator_to_base[0].translation = Vec3F(-42.0, 0.5, 32.0) + base_offset

        self.actuator_to_base[1].rotation = QuaternionF.from_axis_angle(Vec3F(1.0, 0.0, 1.0), 180.0 * DEG2RAD)
        self.actuator_to_base[1].translation = Vec3F(32.0, -42.0, 0.5) + base_offset

        self.actuator_to_base[2].rotation = QuaternionF.from_axis_angle(Vec3F(-1.0, 0.0, 0.0), 90.0 * DEG2RAD)
        self.actuator_to_base[2].translation = Vec3F(0.5, 32.0, -42.0) + base_offset

        self.rotor_angle_offset[0] = 46.2 * DEG2RAD
        self.rotor_angle_offset[1] = 46.2 * DEG2RAD
        self.rotor_angle_offset[2] = 46.2 * DEG2RAD

        for i in range(3):
            self.base_to_actuator[i] = self.actuator_to_base[i].inverse()

    def arm_attachment_point(self, joint_idx, rotor_angle):
        rotor_angle -= self.rotor_angle_offset[joint_idx]
        return Vec3F(math.cos(rotor_angle), -math.sin(rotor_angle), 0.0) * self.rotor_radius

    def forward(self, joint_positions):
        """
        Compute end-effector pose from joint positions (radians).
        Returns (success, Pose6DF)
        """
        arm_attachment_points = []
        for i in range(3):
            p = self.arm_attachment_point(i, joint_positions[i])
            p = self.actuator_to_base[i].transformPoint(p)
            # Adjust for ee attachment point
            arm_attachment_points.append(p - self.ee_attachment_points[i])

        intersections = three_sphere_intersection(
            arm_attachment_points[0], self.arm_length,
            arm_attachment_points[1], self.arm_length,
            arm_attachment_points[2], self.arm_length
        )

        if not intersections:
            return False, Pose6DF()

        # Select solution with greater x (heuristic from C++)
        q = intersections[0] if intersections[0].x > intersections[1].x else intersections[1]

        return True, Pose6DF(translation=q)

    def inverse(self, pose):
        """
        Compute joint positions (radians) from end-effector pose.
        Returns (success, [theta1, theta2, theta3])
        """
        joint_positions = [0.0] * 3
        for i in range(3):
            p = pose.transformPoint(self.ee_attachment_points[i])
            p = self.base_to_actuator[i].transformPoint(p)

            intersections = circle_sphere_intersection(self.rotor_radius, p, self.arm_length)
            if not intersections:
                return False, []

            # Select solution based on x-position (in actuator coords)
            q = intersections[0] if intersections[0].x > intersections[1].x else intersections[1]

            angle = -math.atan2(q.y, q.x)
            joint_positions[i] = self.rotor_angle_offset[i] + angle

        return True, joint_positions

if __name__ == "__main__":
    # Simple test to verify port matches C++ test output structure
    model = KinematicModel_Delta3D()

    print("# Forward Kinematic")
    joint_pos = [45 * DEG2RAD] * 3
    success, pose = model.forward(joint_pos)
    if success:
        p = pose.translation
        print(f"{p.x:.6f} {p.y:.6f} {p.z:.6f}")
    else:
        print("Forward Kinematic Failed")

    print("\n# Inverse Kinematic")
    target_pose = Pose6DF(Vec3F(0.0, 0.0, 0.0), QuaternionF())
    success, joints = model.inverse(target_pose)
    if success:
        print(f"{joints[0]/DEG2RAD:.6f} {joints[1]/DEG2RAD:.6f} {joints[2]/DEG2RAD:.6f}")
    else:
        print("Inverse Kinematic Failed")
