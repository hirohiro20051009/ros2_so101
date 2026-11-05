#!/usr/bin/env python3
"""diffik_go.py: 多点の1本軌道で目標へ動く go() と、再送つきの visit()。
設定変更なし。走行中は 0.1 秒ごとに des-meas と測定Zを監視し、超えたらゴールをキャンセルする。
(open_loop 下でキャンセル後に腕がどこで止まるかは未検証。手は電源に置いておくこと。)"""
import time
import numpy as np
import rclpy
from control_msgs.action import FollowJointTrajectory
from trajectory_msgs.msg import JointTrajectoryPoint
from so101_diffik import DiffIK, ARM, Z_MIN, MAX_OFFSET, TICK, SETTLE
from diffik_path import plan, dur, MAX_DIST, MAX_SEG_DQ


def _send(n, goals, times):
    """軌道を送り、ゴールハンドルを返す（拒否なら None）。"""
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
    h = fut.result()
    return h if h.accepted else None


def _cancel(n, h):
    fut = h.cancel_goal_async()
    rclpy.spin_until_future_complete(n, fut)


def go(n, target, nseg=10, speed=0.01):
    """現在の測定値から target へ1本軌道で動く。dict を返す。
    ok=False のときは why に理由（送信前の中止、または走行中の中止）。"""
    target = np.asarray(target, dtype=float)
    n.spin(0.5)
    q0, d0 = n.q(), n.d()
    gap0 = d0 - q0
    p0 = n.fk(q0)
    dist = float(np.linalg.norm(target - p0))
    T = max(2.0, dist / speed)
    r = dict(ok=False, why='', p=p0, err_mm=(target - p0) * 1000,
             gap0=gap0 / TICK, gap1=gap0 / TICK, cmd_dq=np.zeros(5),
             meas_dq=np.zeros(5), T=T)
    if target[2] < Z_MIN:
        r['why'] = 'target below Z_MIN'; return r
    if dist > MAX_DIST:
        r['why'] = f'distance {dist*1000:.0f} mm > MAX_DIST'; return r
    if np.any(np.abs(gap0) > MAX_OFFSET):
        r['why'] = 'des-meas too large at start'; return r
    qs, ps, xs = plan(n, q0, p0, target, nseg)
    if ps[:, 2].min() < Z_MIN + 0.01:
        r['why'] = 'path too close to Z_MIN'; return r
    if np.abs(np.diff(qs, axis=0)).max() > MAX_SEG_DQ:
        r['why'] = 'segment dq too large'; return r
    r['cmd_dq'] = (qs[-1] - q0) / TICK
    goals = d0 + (qs[1:] - q0)
    times = T * np.arange(1, nseg + 1) / nseg
    h = _send(n, goals, times)
    if h is None:
        r['why'] = 'goal rejected'; return r
    t_end = time.time() + T + SETTLE
    while time.time() < t_end:
        n.spin(0.1)
        q = n.q()
        gap = n.d() - q
        if np.any(np.abs(gap) > MAX_OFFSET):
            _cancel(n, h)
            r['why'] = 'ABORT: des-meas exceeded during run ' + str(np.round(gap / TICK, 1))
            r['gap1'] = gap / TICK
            return r
        if n.fk(q)[2] < Z_MIN:
            _cancel(n, h)
            r['why'] = 'ABORT: measured Z below Z_MIN during run'
            return r
    p1 = n.record()
    q1 = n.q()
    r.update(ok=True, why='done', p=p1, err_mm=(target - p1) * 1000,
             gap1=(n.d() - q1) / TICK, meas_dq=(q1 - q0) / TICK)
    return r


def visit(n, target, tol=0.003, max_resend=2, **kw):
    """go() を最大 1+max_resend 回。誤差が tol 以下になったら止める。
    戻り値: (最後の go の dict, 送信回数, 最初の gap0)。中止時は ok=False のまま返る。"""
    target = np.asarray(target, dtype=float)
    first_gap0, res = None, None
    for i in range(1 + max_resend):
        res = go(n, target, **kw)
        if first_gap0 is None:
            first_gap0 = res['gap0']
        print(f'   send {i+1}: ok={res["ok"]} {res["why"]} | err(target-meas) mm '
              f'{np.round(res["err_mm"], 1)} norm {np.linalg.norm(res["err_mm"]):.1f} | '
              f'cmd dq {np.round(res["cmd_dq"], 1)} meas dq {np.round(res["meas_dq"], 1)} | '
              f'des-meas {np.round(res["gap1"], 1)}')
        if not res['ok']:
            return res, i + 1, first_gap0
        if np.linalg.norm(res['err_mm']) <= tol * 1000:
            return res, i + 1, first_gap0
        # 指令があるのに測定が動かない（遊びの中）: 同じ再送は効かないので止める
        if i >= 1 and np.abs(res['meas_dq'][:4]).max() < 1.0 and np.abs(res['cmd_dq'][:4]).max() >= 1.0:
            res['why'] = 'done (stuck in play)'
            print('   stuck: 測定が動かないので再送を止めます')
            return res, i + 1, first_gap0
    return res, 1 + max_resend, first_gap0