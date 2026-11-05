#!/usr/bin/env python3
# contact_test.py [scale] [--hover=0.030] [--dry] [--far]
# 机上の tag36h11_1 の base_link 位置の真上 hover[m] へ TCP を動かす (pitch/roll=0/0)。
# 測定TCPが目標に近づくまで、補正した目標を最大3回送る。
import sys, time
import collect_cal as C
import numpy as np, rclpy
from rclpy.time import Time
from rclpy.action import ActionClient
from tf2_ros import Buffer, TransformListener
import so101_ik as A

REF, TAGF, NF = 'base_link', 'tag36h11_1', 10
HOVER = 0.040
for a in sys.argv[1:]:
    if a.startswith('--hover='): HOVER = float(a.split('=')[1])

def read_tag(n, buf, timeout=8.0):
    S = []; last = None; t_end = time.time() + timeout
    while len(S) < NF and time.time() < t_end:
        n.spin(0.05)
        try: t = buf.lookup_transform(REF, TAGF, Time())
        except Exception: continue
        st = t.header.stamp.sec + t.header.stamp.nanosec * 1e-9
        if st == last: continue
        last = st
        if n.get_clock().now().nanoseconds * 1e-9 - st > 1.0: continue
        v = t.transform.translation; r = t.transform.rotation
        nz = [2*(r.x*r.z + r.y*r.w), 2*(r.y*r.z - r.x*r.w), 1 - 2*(r.x*r.x + r.y*r.y)]
        S.append([v.x, v.y, v.z, *nz])
    return np.array(S) if len(S) == NF else None

def main():
    rclpy.init(); n = C.DiffIK()
    np.set_printoptions(precision=2, suppress=True)
    if not n.ready(): print('topics/services not ready'); return
    buf = Buffer(); TransformListener(buf, n)
    S = read_tag(n, buf)
    if S is None: print(f'{TAGF} not seen'); return
    m = S[:, :3].mean(0); nz = S[:, 3:].mean(0); nz /= np.linalg.norm(nz)
    tilt = np.degrees(np.arccos(nz[2]))
    print(f'tag center in {REF} [mm]={m*1000} std[mm]={S[:, :3].std(0)*1000} tilt={tilt:.1f} deg')
    if tilt > 10: print('tag not flat (tilt > 10 deg), abort'); return
    T = np.array([m[0], m[1], m[2] + HOVER])
    print(f'target TCP [mm]={T*1000} (tag center + {HOVER*1000:.0f} mm)')
    if T[2] < C.Z_MIN + 0.010: print('target too low vs Z_MIN, abort'); return
    try: qb = C.q_for(n, T)
    except A.IKError as e: print('IK error:', e); return
    print('goal q[deg]=', np.degrees(qb))
    if C.DRY: print('dry run: 動かしません'); return
    cli = ActionClient(n, C.MoveGroup, 'move_action'); cli.wait_for_server(timeout_sec=10.0)
    Tc = T.copy()
    for k in range(3):
        q0 = n.q(); p0 = n.fk(q0); gap0 = n.d() - q0
        print(f'\n--- send {k+1}/3 corrected target[mm]={Tc*1000} start TCP[mm]={p0*1000}')
        if p0[2] < C.Z_MIN: print('start Z below Z_MIN'); break
        if np.any(np.abs(gap0) > C.MAX_OFFSET): print('des-meas too large at start'); break
        if Tc[2] < C.Z_MIN + 0.010: print('corrected target too low vs Z_MIN, abort'); break
        try: qb = C.q_for(n, Tc)
        except A.IKError as e: print('IK error:', e); break
        pl = C.plan(n, cli, qb)
        if pl is None: break
        P, dur = pl; tz = np.array([n.fk(q)[2] for q in P])
        print(f'PLAN points={len(P)} duration={dur:.2f}s min TCP z={tz.min()*1000:.1f}mm')
        if tz.min() < C.Z_MIN + 0.005: print('plan passes too close to Z_MIN, abort'); break
        if k == 0:
            d = np.linalg.norm(Tc - p0)
            if d > C.MAX_DIST and not C.FAR:
                print(f'WARNING: 目標まで {d*1000:.0f} mm (> {C.MAX_DIST*1000:.0f} mm)')
                if input('このまま実行しますか？ [y/N]: ').strip().lower() != 'y': print('中止'); break
            input('腕の周りが空いていて、手が電源に届く位置なら Enter で実行 (以降は自動, Ctrl-C で中止): ')
        ok, why, mg, zmin = C.execute(n, cli, qb)
        if why.startswith('ABORT') or not ok: print('stop:', why); break
        C.wait_still(n)
        p, R = C.fk_pose(n, n.q())
        err = T - p
        print(f'measured TCP[mm]={p*1000} err(target-TCP)[mm]={err*1000} |err|={np.linalg.norm(err)*1000:.1f}')
        if np.linalg.norm(err) < 0.0015: break
        Tc = Tc + err
    print('\n停止位置で、真上から TCP とタグ中心の水平のずれ(前後 X, 左右 Y)と、TCP の高さをメジャーで測ってください')
    n.destroy_node(); rclpy.shutdown()

if __name__ == '__main__':
    main()
