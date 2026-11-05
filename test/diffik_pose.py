#!/usr/bin/env python3
"""diffik_pose.py: FK の姿勢（回転行列）と、タグ中心の位置の計算。
so101_diffik.py は変更しない（DiffIK の self.cli を使う）。"""
import numpy as np
import rclpy
from moveit_msgs.srv import GetPositionFK
from so101_diffik import ARM, TIP, BASE

# 全関節 0 のときの、TCP からタグ中心へのオフセット（base_link の軸方向）[m]（概算）
D_BASE0 = np.array([-0.055, 0.0, 0.035])


def quat_to_R(x, y, z, w):
    n = x * x + y * y + z * z + w * w
    s = 2.0 / n
    return np.array([
        [1 - s * (y * y + z * z), s * (x * y - z * w), s * (x * z + y * w)],
        [s * (x * y + z * w), 1 - s * (x * x + z * z), s * (y * z - x * w)],
        [s * (x * z - y * w), s * (y * z + x * w), 1 - s * (x * x + y * y)]])


def fk_pose(n, q):
    """DiffIK n と関節角 q (5,) から、TCP の base_link 位置 (3,) と回転行列 (3,3)。"""
    req = GetPositionFK.Request()
    req.header.frame_id = BASE
    req.fk_link_names = [TIP]
    req.robot_state.joint_state.name = ARM
    req.robot_state.joint_state.position = [float(v) for v in q]
    f = n.cli.call_async(req)
    rclpy.spin_until_future_complete(n, f)
    r = f.result()
    if r.error_code.val != 1:
        raise RuntimeError(f'compute_fk error {r.error_code.val}')
    pose = r.pose_stamped[0].pose
    p = np.array([pose.position.x, pose.position.y, pose.position.z])
    o = pose.orientation
    return p, quat_to_R(o.x, o.y, o.z, o.w)


def tag_offset_gripper(n, d_base0=D_BASE0):
    """全関節 0 の姿勢 R0 で、base_link のオフセットを、グリッパー座標系のオフセットに直す。"""
    _, R0 = fk_pose(n, np.zeros(5))
    return R0.T @ d_base0, R0


def tilt_deg(R0, R):
    """全関節 0 の姿勢からの回転角 [deg]（座標軸の取り方によらない）。"""
    c = (np.trace(R0.T @ R) - 1.0) / 2.0
    return float(np.degrees(np.arccos(np.clip(c, -1.0, 1.0))))