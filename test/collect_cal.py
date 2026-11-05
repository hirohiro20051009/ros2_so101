#!/usr/bin/env python3
# collect_cal.py [scale] [--far] [--dry] : 中心+立方体8頂点の9点を1回ずつ移動し、
# 整定後の測定TCP(タグ中心換算)とカメラ座標のタグ位置をCSVに保存 (pitch/roll=0/0)
import os, sys, time, csv, datetime
import numpy as np, rclpy
from rclpy.action import ActionClient
from rclpy.time import Time
from tf2_ros import Buffer, TransformListener
from moveit_msgs.action import MoveGroup
from moveit_msgs.msg import MotionPlanRequest, Constraints, JointConstraint
from so101_diffik import DiffIK, ARM, MAX_OFFSET, TICK
import so101_ik as A
from diffik_pose import fk_pose, tag_offset_gripper

Z_MIN = 0.025           # [m] 許可済み
TOL = 0.001
MAX_DIST = 0.12         # [m]
CENTER = np.array([0.361, 0.0, 0.205]); HW = 0.05
SIGNS = [(-1,-1,-1), (-1,-1,1), (-1,1,1), (-1,1,-1), (1,1,-1), (1,1,1), (1,-1,1), (1,-1,-1)]
CAM, TAG = 'camera_color_optical_frame', 'tag36h11_0'
NF = 10

args = [a for a in sys.argv[1:] if not a.startswith('--')]
FAR = '--far' in sys.argv; DRY = '--dry' in sys.argv
SCALE = float(args[0]) if args else 0.3
WPS = [CENTER] + [CENTER + HW * np.array(s, float) for s in SIGNS]

def q_for(n, T):
    Tc = T.copy()
    for _ in range(3):
        q = A.solve_ik(*Tc, current_q=np.zeros(5), pitch=0.0, roll=0.0)
        p, _ = fk_pose(n, q)
        Tc = Tc - (p - T)
    return q

def send(n, cli, qb, plan_only):
    goal = MoveGroup.Goal(); req = MotionPlanRequest()
    req.group_name = 'arm'; req.num_planning_attempts = 10; req.allowed_planning_time = 5.0
    req.max_velocity_scaling_factor = SCALE; req.max_acceleration_scaling_factor = SCALE
    req.start_state.is_diff = True
    c = Constraints()
    for name, pos in zip(ARM, qb):
        jc = JointConstraint(); jc.joint_name = name; jc.position = float(pos)
        jc.tolerance_above = TOL; jc.tolerance_below = TOL; jc.weight = 1.0
        c.joint_constraints.append(jc)
    req.goal_constraints.append(c); goal.request = req
    goal.planning_options.plan_only = plan_only
    f = cli.send_goal_async(goal); rclpy.spin_until_future_complete(n, f)
    return f.result()

def plan(n, cli, qb):
    h = send(n, cli, qb, True)
    if not h or not h.accepted: print('plan rejected'); return None
    rf = h.get_result_async(); rclpy.spin_until_future_complete(n, rf)
    res = rf.result().result; jt = res.planned_trajectory.joint_trajectory
    if res.error_code.val != 1 or not jt.points: print('plan failed', res.error_code.val); return None
    order = [jt.joint_names.index(j) for j in ARM]
    P = np.array([[p.positions[i] for i in order] for p in jt.points])
    tf = jt.points[-1].time_from_start
    return P, tf.sec + tf.nanosec * 1e-9

def execute(n, cli, qb):
    """戻り値: (ok, why, max_gap, zmin)"""
    h = send(n, cli, qb, False)
    if not h or not h.accepted: return False, 'goal rejected', None, None
    rf = h.get_result_async(); t_end = time.time() + 30.0
    max_gap = np.zeros(5); zmin = 9.9; why = ''
    while not rf.done():
        n.spin(0.1)
        q = n.q(); gap = n.d() - q
        max_gap = np.maximum(max_gap, np.abs(gap)); z = n.fk(q)[2]; zmin = min(zmin, z)
        if np.any(np.abs(gap) > MAX_OFFSET): why = 'ABORT des-meas ' + str(np.round(gap / TICK, 1))
        elif z < Z_MIN: why = f'ABORT measured Z {z*1000:.1f} mm < Z_MIN'
        elif time.time() > t_end: why = 'ABORT timeout 30 s'
        if why:
            c = h.cancel_goal_async(); rclpy.spin_until_future_complete(n, c)
            return False, why, max_gap, zmin
    ok = rf.result().result.error_code.val == 1
    return ok, '' if ok else f'code {rf.result().result.error_code.val}', max_gap, zmin

def wait_still(n, timeout=8.0):
    """q が 0.5 秒間で 1 tick 以内なら True"""
    t_end = time.time() + timeout; hist = []
    while time.time() < t_end:
        n.spin(0.1); hist.append((time.time(), n.q()))
        old = [h for h in hist if hist[-1][0] - h[0] >= 0.5]
        if old and np.max(np.abs(hist[-1][1] - old[-1][1])) / TICK <= 1.0: return True
    return False

def capture(n, buf, timeout=6.0):
    """タグ NF フレーム (stamp 違い、1秒以内) を取る。各フレームで q も読む。失敗は None"""
    S = []; last = None; t_end = time.time() + timeout
    while len(S) < NF and time.time() < t_end:
        n.spin(0.05)
        try: t = buf.lookup_transform(CAM, TAG, Time())
        except Exception: continue
        st = t.header.stamp.sec + t.header.stamp.nanosec * 1e-9
        if st == last: continue
        last = st
        if n.get_clock().now().nanoseconds * 1e-9 - st > 1.0: continue
        v = t.transform.translation; r = t.transform.rotation
        nz = [2*(r.x*r.z + r.y*r.w), 2*(r.y*r.z - r.x*r.w), 1 - 2*(r.x*r.x + r.y*r.y)]  # タグZ軸
        S.append([v.x, v.y, v.z, *nz, *n.q()])
    return np.array(S) if len(S) == NF else None

def main():
    rclpy.init(); n = DiffIK()
    np.set_printoptions(precision=2, suppress=True)
    if not n.ready(): print('topics/services not ready'); return
    buf = Buffer(); TransformListener(buf, n)
    d_g, _ = tag_offset_gripper(n)
    goals = []
    for i, T in enumerate(WPS, 1):
        try: qb = q_for(n, T)
        except A.IKError as e: print(f'point {i} IK error: {e}'); return
        goals.append(qb)
        print(f'{i}: target[mm]={T*1000} goal q[deg]={np.degrees(qb)}')
    print('tag offset (gripper)[mm]=', d_g * 1000, ' scale=', SCALE)
    if DRY: print('dry run: 動かしません'); return
    cli = ActionClient(n, MoveGroup, 'move_action'); cli.wait_for_server(timeout_sec=10.0)
    fn = f'{os.path.dirname(os.path.abspath(__file__))}/cal_{datetime.datetime.now():%Y%m%d_%H%M%S}.csv'
    cols = (['i', 'tx', 'ty', 'tz', 'tcp_x', 'tcp_y', 'tcp_z', 'q1', 'q2', 'q3', 'q4', 'q5',
             'pr_x', 'pr_y', 'pr_z', 'cam_x', 'cam_y', 'cam_z', 'std_x', 'std_y', 'std_z',
             'p2p_x', 'p2p_y', 'p2p_z', 'nrm_x', 'nrm_y', 'nrm_z', 'tag_ok', 'still', 'stable'])
    f = open(fn, 'w', newline=''); w = csv.writer(f); w.writerow(cols); f.flush()
    print('CSV:', fn)
    for i, (T, qb) in enumerate(zip(WPS, goals), 1):
        q0 = n.q(); p0 = n.fk(q0); gap0 = n.d() - q0
        print(f'\n--- point {i}/9 target[mm]={T*1000} start TCP[mm]={p0*1000}')
        dist = np.linalg.norm(T - p0)
        if dist > MAX_DIST and not FAR and i > 1:
            print(f'too far at point {i}: {dist*1000:.0f} mm; stop'); break
        if dist > MAX_DIST and not FAR:
            print(f'WARNING: 目標まで {dist*1000:.0f} mm (> {MAX_DIST*1000:.0f} mm)。大きな移動です。')
            if input('このまま計画を立てますか？ [y/N]: ').strip().lower() != 'y': print('中止'); break
        if p0[2] < Z_MIN: print('start Z below Z_MIN'); break
        if np.any(np.abs(gap0) > MAX_OFFSET): print('des-meas too large at start'); break
        pl = plan(n, cli, qb)
        if pl is None: break
        P, dur = pl; tz = np.array([n.fk(q)[2] for q in P])
        print(f'PLAN points={len(P)} duration={dur:.2f}s min TCP z={tz.min()*1000:.1f}mm')
        if tz.min() < Z_MIN + 0.005: print('plan passes too close to Z_MIN, abort'); break
        if i == 1: input('腕の周りが空いていて、手が電源に届く位置なら Enter で実行 (以降は自動, Ctrl-C で中止): ')
        ok, why, mg, zmin = execute(n, cli, qb)
        if why.startswith('ABORT') or not ok: print('stop:', why); break
        still = wait_still(n)
        S = capture(n, buf); stable = True
        if S is not None:
            d = S[:5, :3].mean(0) - S[5:, :3].mean(0)
            if np.linalg.norm(d) * 1000 > 0.3:
                stable = False; print('tag not stable, recapture'); S2 = capture(n, buf)
                if S2 is not None: S = S2; d = S[:5, :3].mean(0) - S[5:, :3].mean(0); stable = np.linalg.norm(d) * 1000 <= 0.3
        qm = (S[:, 6:11].mean(0) if S is not None else n.q())
        p, R = fk_pose(n, qm); pr = p + R @ d_g
        cam = S[:, :3].mean(0) * 1000 if S is not None else np.full(3, np.nan)
        std = S[:, :3].std(0) * 1000 if S is not None else np.full(3, np.nan)
        p2p = np.ptp(S[:, :3], axis=0) * 1000 if S is not None else np.full(3, np.nan)
        nrm = S[:, 3:6].mean(0) if S is not None else np.full(3, np.nan)
        if S is not None: nrm = nrm / np.linalg.norm(nrm)
        w.writerow([i, *(T*1000).round(3), *(p*1000).round(3), *qm.round(6), *(pr*1000).round(3),
                    *cam.round(3), *std.round(3), *p2p.round(3), *nrm.round(4),
                    int(S is not None), int(still), int(stable)]); f.flush()
        print(f'TCP[mm]={p*1000} err(target-TCP)[mm]={(T-p)*1000} P_robot(tag)[mm]={pr*1000}')
        print(f'cam[mm]={cam} std={std} tag={"OK" if S is not None else "NOT SEEN"} still={still} stable={stable}')
    f.close(); n.destroy_node(); rclpy.shutdown()

if __name__ == '__main__':
    main()
