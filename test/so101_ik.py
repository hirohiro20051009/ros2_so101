"""
SO_ARM101 5-DoF 解析FK / IK + 軽量姿勢ブリッジ
長さ [m]、内部角度 [rad]

方針:
- IK本体は XYZ + pitch + roll
- 外部から pitch / roll が来ればその値を使用
- なければ current_q の FK から現在の pitch / roll を使用
- Quaternion は adapter で標準 RPY へ変換し、 SO-ARM101用 pitch/roll へ渡す
- yaw は5-DoF解析IKの独立高速には利用しない
"""

import math
import numpy as np

# ----- 幾何パラメータ -----
X0, Y0, Z0 = 0.0388353, -8.97657e-9, 0.0624
AP, HP = 0.030, 0.054 # shoulder_pan -> shoulder_lift
L1, L2 = 0.1160, 0.1350
LW, LG = 0.0637, 0.0984
LT = LW + LG

# ----- ゼロ角補正 -----
D2 = math.radians(-10.0) # q2 = 0 のL1垂直時にサーボオフセットの発生分を考慮
D3 = math.radians(0.0) # q2 = 0、q3 = 0時にL2が水平となるように
D4 = 0.0
D5 = 0.0

# ----- 実機の正回転方向 -----
S1 = -1.0 # shoulder_pan: 上から見て時計回り
S2 = -1.0 # shoulder_lift: Y=0の時、+Y側へ見て時計回り
S3 = -1.0 # elbow_flex: Y=0の時、+Y側へ見て時計回り
S4 = -1.0 # wrist_flex: Y=0の時、+Y側へ見て時計回り
S5 =  1.0 # wrist_roll: 反時計周り

# ----- 関節limits -----
Q_MIN = np.array([-1.91986, -1.74533, -1.69, -1.65806, -2.74385])
Q_MAX = np.array([ 1.91986,  1.74533,  1.69,  1.65806,  2.84121])

class  IKError(RuntimeError):
    pass

class IKUnreachableError(IKError):
    pass

class IKJointLimitError(IKError):
    pass

def normalize_angle(a):
    return math.atan2(math.sin(a), math.cos(a))

def within_limits(q):
    q = np.array(q, dtype=float)
    return bool(np.all(q >= Q_MIN) and np.all(q <= Q_MAX))

# ========================================
# 軽量姿勢ブリッジ
# ========================================

def quaternion_to_rpy(qx, qy, qz, qw):
    """ Quaternion(x,y,z,w)を標準的なroll/pitch/yawに変換"""
    n = math.sqrt(qx*qx + qy*qy + qz*qz + qw*qw)
    if n < 1e-12:
        raise ValueError("Quaternion norm is zero")
    
    qx, qy, qz, qw = qx/n, qy/n, qz/n, qw/n

    sinr_cosp = 2.0 * (qw*qx + qy*qz)
    cosr_cosp = 1.0 - 2.0 * (qx*qx + qy*qy)
    roll = math.atan2(sinr_cosp, cosr_cosp)

    sinp = 2.0 * (qw*qy - qz*qx)

    if abs(sinp) >= 1.0:
        pitch = math.copysign(math.pi/2, sinp)
    else:
        pitch = math.asin(sinp)

    siny_cosp = 2.0 * (qw*qz + qx*qy)
    cosy_cosp = 1.0 - 2.0 * (qy*qy + qz*qz)
    yaw = math.atan2(siny_cosp, cosy_cosp)
    return roll, pitch, yaw

def orientation_adapter(quaternion=None, pitch=None, roll=None):
    """
    上位層の姿勢を解析IK用の（pitch, roll）へ橋渡しするadapter

    優先順位
    1) pitch/rollが明示されていれば、その値を使用
    2) 不足分だけQuaternionの標準RPYから補う
    3) どちらもなければNoneのまま返す（solve_ik側で現在姿勢を使う）
    """
    if quaternion is not None:
        if len(quaternion) != 4:
            raise ValueError("quaternion must be [qx, qy, qz, qw]")
        ext_roll, ext_pitch, _ext_yaw = quaternion_to_rpy(*quaternion)
        if pitch is None:
            pitch = ext_pitch
        if roll is None:
            roll = ext_roll

    return pitch, roll

# ========================================
# FK / IK
# ========================================

def fk(q):
    """ q=[q1..q5] -> (XYZ, pitch, roll) """

    q1, q2, q3, q4, q5 = np.array(q, dtype=float)

    theta1 = S1*q1
    alpha1 = S2*q2 + D2 + math.pi/2
    alpha2 = S2*q2 + S3*q3 + D3
    pitch  = S2*q2 + S3*q3 + S4*q4 + D4
    roll   = S5*q5 + D5

    radial = AP + L1*math.cos(alpha1) + L2*math.cos(alpha2) + LT*math.cos(pitch)
    x = X0 + radial*math.cos(theta1)
    y = Y0 + radial*math.sin(theta1)
    z = Z0 + HP + L1*math.sin(alpha1) + L2*math.sin(alpha2) + LT*math.sin(pitch)
    return np.array([x, y, z]), pitch, roll

def ik(x, y, z, pitch, roll):
    """ XYZ + pitch + roll -> limits内の全解析IK候補 """
    dx, dy = x - X0, y - Y0
    theta1 = math.atan2(dy, dx)
    q1 = theta1/S1
    radial = math.hypot(dx, dy)

    rw = radial - AP - LT*math.cos(pitch)
    zw = z - Z0 - HP - LT*math.sin(pitch)

    D = (rw*rw + zw*zw - L1*L1 - L2*L2)/(2.0*L1*L2)

    if abs(D) > 1.0 + 1e-9:
        raise IKUnreachableError("target is geometrically unreachable")
    D = max(-1.0, min(1.0, D))

    valid = []
    s = math.sqrt(max(0.0, 1.0-D*D))

    for elbow_sign in (+1.0, -1.0):
        beta3 = math.atan2(elbow_sign*s, D)
        alpha1 = math.atan2(zw, rw) - math.atan2(L2*math.sin(beta3), L1+L2*math.cos(beta3))
        alpha2 = alpha1 + beta3

        q2 = (alpha1 - math.pi/2 - D2)/S2
        q3 = (alpha2 - D3 - S2*q2)/S3
        q4 = (pitch - D4 - S2*q2 - S3*q3)/S4
        q5 = (roll - D5)/S5

        q = np.array([q1, q2, q3, q4, q5])
        if within_limits(q):
            valid.append(q)

    if not valid:
        raise IKJointLimitError("geometric solution exists, but all candidates exceed joint limits")
    return valid

def select_nearest_solution(solutions, current_q):
    """ elbow ±などの候補から現在関節角に最も近いものを選ぶ """
    current_q = np.asarray(current_q, dtype=float)

    def distance(q):
        dq = [normalize_angle(a-b) for a, b in zip(q, current_q)]
        return np.linalg.norm(dq)
    return min(solutions, key=distance)

def solve_ik(x, y, z, current_q, pitch=None, roll=None, quaternion=None):
    """
    運用方法
    - 外部pitch/rollがあれば最優先
    - Quaternionがあれば、不足しているpitch/rollをadapterで補う
    - 最後まで指定がなければ、現在のqのFK姿勢を維持
    - 指定された姿勢で解けなければIKErrorを返す、勝手な広範囲pitch探索はしない
    """
    pitch, roll = orientation_adapter(quaternion, pitch, roll)

    _, current_pitch, current_roll = fk(current_q)
    if pitch is None:
        pitch = current_pitch
    if roll is None:
        roll = current_roll

    solutions = ik(x, y, z, pitch, roll)
    return select_nearest_solution(solutions, current_q)

if __name__ == "__main__":
    #current_q = np.zeros(5)
    current_q = np.array([0, 0, 0, 0, 0])
    xyz, current_pitch, current_roll = fk(current_q)

    np.set_printoptions(precision=3, suppress=True)

    print("zero XYZ [m] =", xyz)
    print("zero pitch/roll [deg] =", math.degrees(current_pitch), math.degrees(current_roll))

    # 1) pitch / roll 明示
    q_goal = solve_ik(*xyz, current_q=current_q, pitch=0.0, roll=0.0)
    print("explicit orientation [deg] =", np.degrees(q_goal))

    # 2) 姿勢指定なし -> 現在 pitch / rollを利用
    q_goal = solve_ik(*xyz, current_q=current_q)
    print("keep current orientation [deg] =", np.degrees(q_goal))

    # 3) Quaternion入力例: 単位Quaternion -> QPY=0
    q_goal = solve_ik(*xyz, current_q=current_q, quaternion=[0.0, 0.0, 0.0, 1.0])
    print("quaternion input [deg] =", np.degrees(q_goal))
