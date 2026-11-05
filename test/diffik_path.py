#!/usr/bin/env python3
"""diffik_path.py X Y Z [--dry] [--nseg 10] [--speed 0.01] [--refine 0]
現在の測定TCPから目標まで直線。計算だけで関節角の経路を作り、1本の多点軌道で送る。
--dry は送らずに経路だけ表示。--refine N は到達後に move_to を最大N回。"""
import argparse
import numpy as np
import rclpy
from control_msgs.action import FollowJointTrajectory
from trajectory_msgs.msg import JointTrajectoryPoint
from builtin_interfaces.msg import Duration
from so101_diffik import DiffIK, ARM, Z_MIN, MAX_OFFSET, LAM, TICK, SETTLE

MAX_DIST = 0.06      # 現在位置から目標までの上限 [m]
MAX_SEG_DQ = 0.03    # 隣り合う経路点の関節変化の上限 [rad]


def dur(t):
    s = int(t)
    return Duration(sec=s, nanosec=int(round((t - s) * 1e9)))


def plan(n, q0, p0, target, nseg):
    qs, ps, xs = [q0.copy()], [p0.copy()], [p0.copy()]
    q = q0.copy()
    for k in range(1, nseg + 1):
        x = p0 + (target - p0) * k / nseg
        for _ in range(2):
            p = n.fk(q)
            J = n._jac(q, p)
            q = q + J.T @ np.linalg.solve(J @ J.T + LAM**2 * np.eye(3), x - p)
        qs.append(q.copy()); ps.append(n.fk(q)); xs.append(x)
    return np.array(qs), np.array(ps), np.array(xs)


def send_path(n, goals, times):
    N = len(goals)
    g = FollowJointTrajectory.Goal()
    g.trajectory.joint_names = ARM
    for k in range(N):
        if k == 0 or k == N - 1:
            v = np.zeros(5)
        else:
            v = (goals[k + 1] - goals[k - 1]) / (times[k + 1] - times[k - 1])
        pt = JointTrajectoryPoint()
        pt.positions = [float(x) for x in goals[k]]
        pt.velocities = [float(x) for x in v]
        pt.time_from_start = dur(times[k])
        g.trajectory.points.append(pt)
    fut = n.ac.send_goal_async(g)
    rclpy.spin_until_future_complete(n, fut)
    return fut.result().accepted


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument('xyz', nargs=3, type=float)
    ap.add_argument('--dry', action='store_true')
    ap.add_argument('--nseg', type=int, default=10)
    ap.add_argument('--speed', type=float, default=0.01)
    ap.add_argument('--refine', type=int, default=0)
    a = ap.parse_args()
    target = np.array(a.xyz)
    assert 2 <= a.nseg <= 30 and 0.003 <= a.speed <= 0.02 and 0 <= a.refine <= 6
    rclpy.init()
    try:
        n = DiffIK()
        if not n.ready():
            print('必要なトピック/サービス/アクションが使えません'); return
        n.spin(0.5)
        q0, d0 = n.q(), n.d()
        gap0 = d0 - q0
        p0 = n.fk(q0)
        dist = np.linalg.norm(target - p0)
        T = max(2.0, dist / a.speed)
        print('start TCP', np.round(p0, 5), '-> target', np.round(target, 5),
              f'dist {dist*1000:.1f} mm, T {T:.1f} s')
        print('gap(des-meas) [ticks]', np.round(gap0 / TICK, 1))
        if target[2] < Z_MIN: print('中止: target below Z_MIN'); return
        if dist > MAX_DIST: print('中止: 距離が大きすぎます'); return
        if np.any(np.abs(gap0) > MAX_OFFSET): print('中止: des-meas が大きい'); return
        qs, ps, xs = plan(n, q0, p0, target, a.nseg)
        seg = np.abs(np.diff(qs, axis=0)).max()
        res = np.linalg.norm(ps - xs, axis=1).max() * 1000
        print(f'経路: 最大残差 {res:.2f} mm, 最大セグメント関節変化 {seg/TICK:.1f} ticks, '
              f'最低Z {ps[:, 2].min():.4f}')
        for k in range(1, a.nseg + 1):
            print(f' k={k:2d} t={T*k/a.nseg:4.1f}s pred {np.round(ps[k], 4)} '
                  f'dq[ticks] {np.round((qs[k]-q0)/TICK, 1)}')
        if ps[:, 2].min() < Z_MIN + 0.01: print('中止: 経路がZ_MINに近い'); return
        if seg > MAX_SEG_DQ: print('中止: 経路の関節変化が大きい'); return
        if a.dry:
            return
        goals = d0 + (qs[1:] - q0)          # desired + dq (gap を保つ)
        times = T * np.arange(1, a.nseg + 1) / a.nseg
        if not send_path(n, goals, times):
            print('goal rejected'); return
        n.spin(T + SETTLE)
        p1 = n.record()
        q1 = n.q()
        print('measured TCP', np.round(p1, 5), '| err to target [mm]',
              np.round((target - p1) * 1000, 1), f'norm {np.linalg.norm(target-p1)*1000:.1f}')
        print('pred dTCP [mm]', np.round((ps[-1] - p0) * 1000, 1),
              '| actual', np.round((p1 - p0) * 1000, 1))
        print('cmd dq [ticks]', np.round((qs[-1] - q0) / TICK, 1),
              '| meas dq', np.round((q1 - q0) / TICK, 1),
              '| des-meas [ticks]', np.round((n.d() - q1) / TICK, 1))
        if a.refine > 0:
            ok, p, why = n.move_to(target, max_steps=a.refine, tol=0.003)
            print('refine:', ok, np.round(p, 5), why)
    finally:
        rclpy.shutdown()


if __name__ == '__main__':
    main()
