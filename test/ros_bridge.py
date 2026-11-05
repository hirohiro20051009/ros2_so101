#!/usr/bin/env python3
# -------------------
# --- ros_bridge.py ---
# -------------------
#
# KDL (position_only_ik: true) + ブリッジ側姿勢選択
#
# 【IK 解決フロー】
#   フェーズ1: 現在関節値 Seed で試行
#   フェーズ2: ランダム Seed で IK_RETRIES 回試行（グローバル探索）
#   フェーズ3: 最良解近傍にノイズを加えた Seed で IK_LOCAL_RETRIES 回試行（ローカル探索）
#              ※ 姿勢指定ありの場合のみ実行。姿勢誤差を集中的に縮める。
#
# 【コマンド例】
#   {"pose": "home"}
#   {"x": 0.3, "y": 0.0, "z": 0.2}
#   {"x": 0.3, "y": 0.0, "z": 0.2, "qx": 0.0, "qy": 1.0, "qz": 0.0, "qw": 0.0}

import rclpy
from rclpy.node import Node
from rclpy.action import ActionClient
from moveit_msgs.action import MoveGroup
from moveit_msgs.srv import GetPositionIK, GetPositionFK
from moveit_msgs.msg import (
    PositionIKRequest, MotionPlanRequest,
    Constraints, JointConstraint
)
from sensor_msgs.msg import JointState
import socket
import json
import math
import random
import time


class ROSBridge(Node):

    GROUP_NAME = 'arm'
    BASE_FRAME = 'base_link'
    EE_FRAME   = 'gripper_frame_link'

    ARM_JOINT_NAMES = [
        'shoulder_pan', 'shoulder_lift', 'elbow_flex',
        'wrist_flex', 'wrist_roll',
    ]
    JOINT_LIMITS = [
        (-math.pi/2,   math.pi/2),
        (-math.pi/2,   math.pi/2),
        (-math.pi/2,   math.pi/2),
        (-math.pi/2,   math.pi/2),
        (-math.pi*0.9, math.pi*0.9),
    ]
    ARM_POSES = {
        'home':   [0.0, -1.57, 1.57, 0.8,  0.0],
        'start':  [0.0,  0.0,  0.0,  1.57, 0.0],
        'center': [0.0,  0.0,  0.0,  0.0,  0.0],
    }
    HAND_POSES = {'open': [0.8], 'close': [0.0]}

    # -------------------------------------------------------
    # 調整パラメータ
    # -------------------------------------------------------
    IK_TIMEOUT_SEC    = 1.0
    IK_RETRIES        = 10    # グローバル探索: ランダムSeed 試行回数
    IK_LOCAL_RETRIES  = 20    # ローカル探索: 最良解近傍の試行回数（姿勢指定時のみ）
    LOCAL_NOISE_START = 0.3   # ローカル探索の初期ノイズ幅[rad]
    LOCAL_NOISE_END   = 0.05  # ローカル探索の最終ノイズ幅[rad]（徐々に絞る）
    POS_TOLERANCE     = 0.005 # 位置誤差の早期終了閾値[m]
    ORI_TOLERANCE     = math.radians(10.0)  # 姿勢誤差の早期終了閾値[rad]
    ORI_WEIGHT        = 0.1   # score = pos_dist + ORI_WEIGHT * ori_diff

    def __init__(self):
        super().__init__('ros_bridge')
        self._ik_client     = self.create_client(GetPositionIK,  '/compute_ik')
        self._fk_client     = self.create_client(GetPositionFK,  '/compute_fk')
        self._action_client = ActionClient(self, MoveGroup, 'move_action')
        self.current_js     = None

        self.create_subscription(
            # JointState, '/joint_states',
            JointState, '/follower/joint_states',
            lambda m: setattr(self, 'current_js', m), 10
        )

        self.get_logger().info('サービス待機中...')
        for c in [self._ik_client, self._fk_client]:
            if not c.wait_for_service(timeout_sec=10.0):
                raise RuntimeError(f'{c.srv_name} が見つかりません')
        if not self._action_client.wait_for_server(timeout_sec=10.0):
            raise RuntimeError('MoveGroup Action Server が見つかりません')

        t = time.time()
        while not self.current_js and time.time() - t < 10.0:
            rclpy.spin_once(self, timeout_sec=0.1)
        if not self.current_js:
            raise RuntimeError('joint_states が届きません')

        self.get_logger().info('ROS Bridge Ready')

    # ===================================================
    # JointState ユーティリティ
    # ===================================================

    def _make_js(self, positions=None):
        js = JointState()
        js.header.stamp = self.get_clock().now().to_msg()
        if positions is not None:
            js.name     = list(self.ARM_JOINT_NAMES)
            js.position = list(positions)
        else:
            js.name     = list(self.ARM_JOINT_NAMES)
            js.position = [
                self.current_js.position[self.current_js.name.index(n)]
                for n in self.ARM_JOINT_NAMES
            ]
        for n, p in zip(self.current_js.name, self.current_js.position):
            if n not in js.name:
                js.name.append(n); js.position.append(p)
        return js

    def _random_js(self, x=None, y=None):
        """ランダムSeed生成。x/y が指定された場合は shoulder_pan を arctan(y/x) に固定"""
        positions = [random.uniform(lo, hi) for lo, hi in self.JOINT_LIMITS]
        if x is not None and y is not None:
            positions[0] = math.atan2(y, x)  # shoulder_pan を方向に合わせる
        return self._make_js(positions)

    def _perturb_js(self, base_js, noise_std):
        """base_js の ARM 関節にガウスノイズを加えた JointState を返す"""
        base_pos = [base_js.position[base_js.name.index(n)] for n in self.ARM_JOINT_NAMES]
        new_pos  = []
        for p, (lo, hi) in zip(base_pos, self.JOINT_LIMITS):
            perturbed = p + random.gauss(0, noise_std)
            new_pos.append(max(lo, min(hi, perturbed)))  # クリッピング
        return self._make_js(new_pos)

    # ===================================================
    # 姿勢ユーティリティ
    # ===================================================

    @staticmethod
    def _quat_angle_diff(q1, q2):
        dot = min(abs(sum(a*b for a,b in zip(q1,q2))), 1.0)
        return 2.0 * math.acos(dot)

    # ===================================================
    # FK / IK
    # ===================================================

    def _call_fk(self, js):
        req = GetPositionFK.Request()
        req.header.frame_id = self.BASE_FRAME
        req.fk_link_names   = [self.EE_FRAME]
        req.robot_state.joint_state = js
        f = self._fk_client.call_async(req)
        rclpy.spin_until_future_complete(self, f)
        res = f.result()
        if res and res.error_code.val == 1:
            p = res.pose_stamped[0].pose
            return (p.position.x, p.position.y, p.position.z,
                    p.orientation.x, p.orientation.y,
                    p.orientation.z, p.orientation.w)
        return None

    def _call_ik_once(self, x, y, z, seed_js):
        req = PositionIKRequest()
        req.group_name       = self.GROUP_NAME
        req.ik_link_name     = self.EE_FRAME
        req.avoid_collisions = False
        req.timeout.sec      = int(self.IK_TIMEOUT_SEC)
        req.timeout.nanosec  = 0
        req.robot_state.joint_state = seed_js
        req.pose_stamped.header.frame_id    = self.BASE_FRAME
        req.pose_stamped.pose.position.x    = x
        req.pose_stamped.pose.position.y    = y
        req.pose_stamped.pose.position.z    = z
        req.pose_stamped.pose.orientation.w = 1.0
        srv = GetPositionIK.Request(); srv.ik_request = req
        f = self._ik_client.call_async(srv)
        rclpy.spin_until_future_complete(self, f)
        res = f.result()
        return res.solution.joint_state if (res and res.error_code.val == 1) else None

    def _eval(self, sol, x, y, z, t_ori):
        """FK 検証してスコアを返す。失敗時は None"""
        fk = self._call_fk(sol)
        if fk is None:
            return None, None, None
        fx, fy, fz, fqx, fqy, fqz, fqw = fk
        pos_dist = math.sqrt((fx-x)**2 + (fy-y)**2 + (fz-z)**2)
        if t_ori:
            ori_diff = self._quat_angle_diff(t_ori, [fqx, fqy, fqz, fqw])
            score    = pos_dist + self.ORI_WEIGHT * ori_diff
        else:
            ori_diff = None
            score    = pos_dist
        return pos_dist, ori_diff, score

    # ===================================================
    def get_current_ee_pose(self):
        """FK で現在の EE 位置・姿勢を (x,y,z,qx,qy,qz,qw) で返す。失敗時は None"""
        return self._call_fk(self._make_js())

    def compute_ik(self, x=None, y=None, z=None, qx=None, qy=None, qz=None, qw=None, seed_joints=None):
        """
        XYZ (+姿勢) を指定して IK を解く。

        x/y/z が None の場合は現在の EE 位置を使用（姿勢のみ変更モード）。
        姿勢指定なし: 位置誤差のみで最良解を選ぶ
        姿勢指定あり: グローバル探索 → ローカル探索で姿勢誤差を縮める
        """
        # XYZ が省略された場合は現在の EE 位置を使用
        if x is None or y is None or z is None:
            curr = self.get_current_ee_pose()
            if curr is None:
                self.get_logger().error('現在の EE 位置を取得できません')
                return None
            x = curr[0] if x is None else x
            y = curr[1] if y is None else y
            z = curr[2] if z is None else z
            self.get_logger().info(
                f'XYZ 省略 → 現在位置を使用: ({x:.3f},{y:.3f},{z:.3f})'
            )

        has_ori  = all(v is not None for v in [qx, qy, qz, qw])
        t_ori    = [qx, qy, qz, qw] if has_ori else None
        mode_str = 'XYZ+姿勢' if has_ori else 'XYZのみ'

        self.get_logger().info(
            f'IK開始 [{mode_str}]: target=({x:.3f},{y:.3f},{z:.3f})'
            + (f' ori=({qx:.3f},{qy:.3f},{qz:.3f},{qw:.3f})' if has_ori else '')
        )

        best_js    = None
        best_score = float('inf')
        best_pos   = None
        best_ori   = None

        def try_seed(seed):
            nonlocal best_js, best_score, best_pos, best_ori
            sol = self._call_ik_once(x, y, z, seed)
            if sol is None:
                return False
            pos_dist, ori_diff, score = self._eval(sol, x, y, z, t_ori)
            if score is None:
                return False
            if score < best_score:
                best_score = score
                best_js    = sol
                best_pos   = pos_dist
                best_ori   = ori_diff
            pos_ok = pos_dist < self.POS_TOLERANCE
            ori_ok = (not has_ori) or (ori_diff is not None and ori_diff < self.ORI_TOLERANCE)
            return pos_ok and ori_ok

        # ── フェーズ0: 外部指定 seed_joints（学習モデル等）────────
        if seed_joints is not None:
            seed_js = self._make_js(seed_joints)
            if seed_js and try_seed(seed_js):
                self._log_result('外部Seed', best_pos, best_ori, early=True)
                return best_js

        # ── フェーズ1: 現在関節値 Seed（近場移動で姿勢の連続性を保つ）──
        if try_seed(self._make_js()):
            self._log_result('現在Seed', best_pos, best_ori, early=True)
            return best_js

        # ── フェーズ2: グローバル探索（ランダムSeed、shoulder_pan は arctan(y/x) 固定）──
        for _ in range(self.IK_RETRIES):
            if try_seed(self._random_js(x, y)):
                self._log_result('グローバル探索', best_pos, best_ori, early=True)
                return best_js

        self._log_result('グローバル探索', best_pos, best_ori, early=False)

        # ── フェーズ3: ローカル探索（姿勢指定ありのみ）──────────
        if has_ori and best_js is not None:
            for i in range(self.IK_LOCAL_RETRIES):
                # ノイズ幅を徐々に絞る（焼きなまし的）
                t     = i / max(self.IK_LOCAL_RETRIES - 1, 1)
                noise = self.LOCAL_NOISE_START * (1-t) + self.LOCAL_NOISE_END * t
                seed  = self._perturb_js(best_js, noise)
                if try_seed(seed):
                    self._log_result('ローカル探索', best_pos, best_ori, early=True)
                    return best_js

            self._log_result('ローカル探索', best_pos, best_ori, early=False)

        if best_js is not None:
            return best_js

        self.get_logger().error('IK: 全試行失敗')
        return None

    def _log_result(self, phase, pos_dist, ori_diff, early):
        if pos_dist is None:
            return
        ori_str = f', 姿勢誤差={math.degrees(ori_diff):.1f}°' if ori_diff is not None else ''
        tag     = '早期終了' if early else '完走'
        self.get_logger().info(
            f'[{phase}] {tag}: 位置誤差={pos_dist*1000:.2f}mm{ori_str}'
        )

    # ===================================================
    # MoveGroup 実行
    # ===================================================

    def _execute(self, joint_solution):
        goal = MoveGroup.Goal()
        req  = MotionPlanRequest()
        req.group_name                      = self.GROUP_NAME
        req.num_planning_attempts           = 10
        req.allowed_planning_time           = 5.0
        req.max_velocity_scaling_factor     = 0.3
        req.max_acceleration_scaling_factor = 0.3
        req.start_state.is_diff             = True

        constraints = Constraints()
        for name, pos in zip(joint_solution.name, joint_solution.position):
            if name not in self.ARM_JOINT_NAMES:
                continue
            jc = JointConstraint()
            jc.joint_name      = name
            jc.position        = pos
            jc.tolerance_above = 0.01
            jc.tolerance_below = 0.01
            jc.weight          = 1.0
            constraints.joint_constraints.append(jc)

        req.goal_constraints.append(constraints)
        goal.request = req

        f = self._action_client.send_goal_async(goal)
        rclpy.spin_until_future_complete(self, f)
        handle = f.result()
        if not handle or not handle.accepted:
            self.get_logger().error('ゴールが拒否されました')
            return False

        rf = handle.get_result_async()
        rclpy.spin_until_future_complete(self, rf)
        res = rf.result()
        ok  = (res.status == 4 and res.result.error_code.val == 1)
        if not ok:
            self.get_logger().error(
                f'実行失敗: status={res.status}, code={res.result.error_code.val}'
            )
        return ok

    def move_to_pose(self, x=None, y=None, z=None,
                     qx=None, qy=None, qz=None, qw=None, seed_joints=None):
        js = self.compute_ik(x, y, z, qx, qy, qz, qw, seed_joints=seed_joints)
        if js is None:
            return False
        return self._execute(js)

    def move_to_named(self, name):
        if name in self.ARM_POSES:
            group, names, positions = self.GROUP_NAME, self.ARM_JOINT_NAMES, self.ARM_POSES[name]
        elif name in self.HAND_POSES:
            group, names, positions = 'hand', ['gripper'], self.HAND_POSES[name]
        else:
            self.get_logger().error(f'未定義ポーズ: {name}')
            return False

        self.get_logger().info(f'名前付きポーズへ移動: {name}')
        goal = MoveGroup.Goal()
        req  = MotionPlanRequest()
        req.group_name                      = group
        req.num_planning_attempts           = 10
        req.allowed_planning_time           = 5.0
        req.max_velocity_scaling_factor     = 0.3
        req.max_acceleration_scaling_factor = 0.3
        req.start_state.is_diff             = True

        constraints = Constraints()
        for n, p in zip(names, positions):
            jc = JointConstraint()
            jc.joint_name      = n
            jc.position        = p
            jc.tolerance_above = 0.01
            jc.tolerance_below = 0.01
            jc.weight          = 1.0
            constraints.joint_constraints.append(jc)

        req.goal_constraints.append(constraints)
        goal.request = req

        f = self._action_client.send_goal_async(goal)
        rclpy.spin_until_future_complete(self, f)
        handle = f.result()
        if not handle or not handle.accepted:
            return False
        rf = handle.get_result_async()
        rclpy.spin_until_future_complete(self, rf)
        res = rf.result()
        return res.status == 4 and res.result.error_code.val == 1

    # ===================================================
    # TCP サーバー
    # ===================================================

    def start_server(self, port=9999):
        with socket.socket(socket.AF_INET, socket.SOCK_STREAM) as s:
            s.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
            s.bind(('0.0.0.0', port))
            s.listen()
            self.get_logger().info(f'TCP Server port:{port} 待機中...')

            while rclpy.ok():
                conn, addr = s.accept()
                self.get_logger().info(f'接続: {addr}')
                with conn:
                    try:
                        data = conn.recv(4096).decode()
                        if not data:
                            continue
                        cmd = json.loads(data)
                        self.get_logger().info(f'受信: {cmd}')

                        pose_name = cmd.get('pose') or cmd.get('named_target')
                        if pose_name:
                            success = self.move_to_named(pose_name)
                        else:
                            success = self.move_to_pose(
                                x=cmd.get('x'),  y=cmd.get('y'),  z=cmd.get('z'),
                                qx=cmd.get('qx'), qy=cmd.get('qy'),
                                qz=cmd.get('qz'), qw=cmd.get('qw'),
                                seed_joints=cmd.get('seed_joints'),
                            )

                        # 実行後の FK で実際の EE 位置・姿勢を取得してレスポンスに含める
                        resp = {'success': success}
                        fk = self.get_current_ee_pose()
                        if fk:
                            resp['fk'] = {
                                'x': round(fk[0], 4), 'y': round(fk[1], 4), 'z': round(fk[2], 4),
                                'qx': round(fk[3], 4), 'qy': round(fk[4], 4),
                                'qz': round(fk[5], 4), 'qw': round(fk[6], 4),
                            }
                            # 目標座標が指定されている場合は誤差も計算
                            tx, ty, tz = cmd.get('x'), cmd.get('y'), cmd.get('z')
                            if all(v is not None for v in [tx, ty, tz]):
                                import math
                                resp['pos_error_mm'] = round(
                                    math.sqrt((fk[0]-tx)**2 + (fk[1]-ty)**2 + (fk[2]-tz)**2) * 1000, 2
                                )
                            tqx, tqy, tqz, tqw = cmd.get('qx'), cmd.get('qy'), cmd.get('qz'), cmd.get('qw')
                            if all(v is not None for v in [tqx, tqy, tqz, tqw]):
                                dot = min(abs(fk[3]*tqx + fk[4]*tqy + fk[5]*tqz + fk[6]*tqw), 1.0)
                                resp['ori_error_deg'] = round(math.degrees(2.0 * math.acos(dot)), 1)

                        conn.sendall(json.dumps(resp).encode())

                    except Exception as e:
                        import traceback; traceback.print_exc()
                        conn.sendall(json.dumps({'success': False, 'error': str(e)}).encode())


def main():
    rclpy.init()
    node = ROSBridge()
    try:
        node.start_server(port=9999)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()