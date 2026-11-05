#!/usr/bin/env python3
"""so101_diffik.py -- 位置のみDiff-IK（設定変更なし）
CLI: python3 so101_diffik.py X Y Z [max_steps]   (base_link の絶対座標 [m])
API: n = DiffIK(); ok, p, why = n.move_to([x, y, z]); p_meas = n.record()"""
import sys, time
import numpy as np
import rclpy
from rclpy.node import Node
from rclpy.action import ActionClient
from sensor_msgs.msg import JointState
from control_msgs.action import FollowJointTrajectory
from control_msgs.msg import JointTrajectoryControllerState
from trajectory_msgs.msg import JointTrajectoryPoint
from builtin_interfaces.msg import Duration
from moveit_msgs.srv import GetPositionFK

ARM = ['shoulder_pan', 'shoulder_lift', 'elbow_flex', 'wrist_flex', 'wrist_roll']
TIP, BASE = 'gripper_frame_link', 'base_link'
Z_MIN, MAX_OFFSET, MAX_DQ, LAM, EPS = 0.06, 0.08, 0.05, 0.01, 1e-4
STEP_T, SETTLE = 2.0, 1.5
TICK = 2 * np.pi / 4095


class DiffIK(Node):
    def __init__(self):
        super().__init__('so101_diffik')
        self.js = self.des = None
        self.create_subscription(JointState, '/follower/joint_states', self._cb_js, 10)
        self.create_subscription(JointTrajectoryControllerState,
                                 '/follower/arm_controller/state', self._cb_st, 10)
        self.cli = self.create_client(GetPositionFK, '/compute_fk')
        self.ac = ActionClient(self, FollowJointTrajectory,
                               '/follower/arm_controller/follow_joint_trajectory')

    def _cb_js(self, m): self.js = dict(zip(m.name, m.position))
    def _cb_st(self, m): self.des = dict(zip(m.joint_names, m.desired.positions))

    def spin(self, s):
        end = time.time() + s
        while time.time() < end:
            rclpy.spin_once(self, timeout_sec=0.01)

    def ready(self):
        self.spin(1.0)
        return (self.js is not None and self.des is not None
                and self.cli.wait_for_service(timeout_sec=3.0)
                and self.ac.wait_for_server(timeout_sec=3.0))

    def q(self): return np.array([self.js[j] for j in ARM])
    def d(self): return np.array([self.des[j] for j in ARM])

    def fk(self, q):
        req = GetPositionFK.Request()
        req.header.frame_id = BASE
        req.fk_link_names = [TIP]
        req.robot_state.joint_state.name = ARM
        req.robot_state.joint_state.position = [float(v) for v in q]
        f = self.cli.call_async(req)
        rclpy.spin_until_future_complete(self, f)
        r = f.result()
        if r.error_code.val != 1:
            raise RuntimeError(f'compute_fk error {r.error_code.val}')
        p = r.pose_stamped[0].pose.position
        return np.array([p.x, p.y, p.z])

    def _jac(self, q, p0):
        J = np.zeros((3, 5))
        for i in range(5):
            dq = np.zeros(5); dq[i] = EPS
            J[:, i] = (self.fk(q + dq) - p0) / EPS
        return J

    def _send(self, goal_q):
        g = FollowJointTrajectory.Goal()
        g.trajectory.joint_names = ARM
        p = JointTrajectoryPoint()
        p.positions = [float(v) for v in goal_q]
        p.time_from_start = Duration(sec=int(STEP_T), nanosec=int((STEP_T % 1) * 1e9))
        g.trajectory.points = [p]
        fut = self.ac.send_goal_async(g)
        rclpy.spin_until_future_complete(self, fut)
        return fut.result().accepted

    def record(self):
        """測定値のFKによるTCP位置 [m] (base_link)。キャリブレーションの P_robot に使う。"""
        self.spin(0.5)
        return self.fk(self.q())

    def move_to(self, target, max_steps=10, tol=0.002, max_dist=0.10):
        """戻り値: (到達したか, 最後の測定TCP, 理由)"""
        target = np.asarray(target, float)
        self.spin(0.3)
        p = self.fk(self.q())
        if target[2] < Z_MIN:
            return False, p, 'target below Z_MIN'
        if np.linalg.norm(target - p) > max_dist:
            return False, p, f'target farther than {max_dist} m from current TCP'
        for k in range(1, max_steps + 1):
            self.spin(0.3)
            q, d = self.q(), self.d()
            gap = d - q
            if np.any(np.abs(gap) > MAX_OFFSET):
                return False, p, f'desired-measured too large {np.round(gap, 4)}'
            p = self.fk(q)
            err = target - p
            if np.linalg.norm(err) < tol:
                return True, p, f'reached in {k - 1} steps'
            J = self._jac(q, p)
            dq = J.T @ np.linalg.solve(J @ J.T + LAM**2 * np.eye(3), err)
            if np.linalg.norm(dq) > MAX_DQ:
                dq *= MAX_DQ / np.linalg.norm(dq)
            p_pred = self.fk(q + dq)
            if p_pred[2] < Z_MIN:
                return False, p, 'predicted TCP below Z_MIN'
            if not self._send(d + dq):
                return False, p, 'goal rejected'
            self.spin(STEP_T + SETTLE)
            q1 = self.q()
            p = self.fk(q1)
            print(f'[{k}] err_before {np.round(err*1000,1)} mm | actual dTCP '
                  f'{np.round((p-(target-err))*1000,1)} | err_after {np.round((target-p)*1000,1)} mm | '
                  f'cmd dq {np.round(dq/TICK,1)} | '
                  f'meas dq [ticks] {np.round((q1-q)/TICK,1)} | des-meas lift '
                  f'{(self.d()-q1)[1]:+.4f} rad')
            if p[2] < Z_MIN:
                return False, p, 'measured TCP below Z_MIN'
        return bool(np.linalg.norm(target - p) < tol), p, 'max_steps'


def main():
    t = [float(x) for x in sys.argv[1:4]]
    steps = int(sys.argv[4]) if len(sys.argv) > 4 else 10
    assert len(t) == 3 and 1 <= steps <= 12
    rclpy.init()
    n = DiffIK()
    if not n.ready():
        print('必要なトピック/サービス/アクションが使えません'); return
    ok, p, why = n.move_to(t, max_steps=steps, tol=0.005)
    print('ok' if ok else 'not reached', np.round(p, 5), why)
    rclpy.shutdown()


if __name__ == '__main__':
    main()
