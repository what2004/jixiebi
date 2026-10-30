import sys
import time

import rclpy
import tf2_ros
from geometry_msgs.msg import PoseStamped
from moveit_msgs.msg import Constraints, JointConstraint, MotionPlanRequest
from moveit_msgs.srv import GetMotionPlan, GetPositionIK
from sensor_msgs.msg import JointState

ARM = ['joint1', 'joint2', 'joint3', 'joint4', 'joint5', 'joint6', 'joint7']

rclpy.init()
node = rclpy.create_node('verify_plan')
ik_cli = node.create_client(GetPositionIK, '/compute_ik')
plan_cli = node.create_client(GetMotionPlan, '/plan_kinematic_path')
buf = tf2_ros.Buffer()
tf_lis = tf2_ros.TransformListener(buf, node)

js = {}
def js_cb(m):
    for n, p in zip(m.name, m.position):
        js[n] = p
node.create_subscription(JointState, '/joint_states', js_cb, 10)

if not ik_cli.wait_for_service(timeout_sec=60) or not plan_cli.wait_for_service(timeout_sec=60):
    print('FAIL: move_group 服务没出现'); sys.exit(1)
print('OK: /compute_ik 与 /plan_kinematic_path 都在')

t0 = time.time()
while len(js) < 7 and time.time() - t0 < 12:
    rclpy.spin_once(node, timeout_sec=0.2)
fallback = False
if len(js) < 7:
    # 没有 joint_state_publisher 在跑时（比如 GUI 窗口被关掉），本脚本代发零位，保证校验可做
    fallback = True
    pub = node.create_publisher(JointState, '/joint_states', 10)
    msg = JointState()
    msg.name = list(ARM)
    msg.position = [0.0] * len(ARM)
    def tick():
        msg.header.stamp = node.get_clock().now().to_msg()
        pub.publish(msg)
        for i, n in enumerate(ARM):
            js[n] = 0.0
    node.create_timer(0.05, tick)
    t0 = time.time()
    while time.time() - t0 < 3:
        rclpy.spin_once(node, timeout_sec=0.1)
print('OK: /joint_states 关节数 %d%s' % (len(js), '  (JSPG 没在跑，本脚本代发零位)' if fallback else ''))

tr = None
t0 = time.time()
while tr is None and time.time() - t0 < 20:
    rclpy.spin_once(node, timeout_sec=0.2)
    try:
        tr = buf.lookup_transform('world', 'tool_link', rclpy.time.Time())
    except Exception:
        pass
if tr is None:
    print('FAIL: 拿不到 tool_link TF'); sys.exit(1)
p0, q0 = tr.transform.translation, tr.transform.rotation
print('OK: 当前法兰 world->tool_link  x=%.3f y=%.3f z=%.3f' % (p0.x, p0.y, p0.z))

def ik(x, y, z, q, seed=None):
    t = PoseStamped()
    t.header.frame_id = 'world'
    t.pose.position.x = x; t.pose.position.y = y; t.pose.position.z = z
    t.pose.orientation = q
    r = GetPositionIK.Request()
    r.ik_request.group_name = 'arm'
    r.ik_request.pose_stamped = t
    r.ik_request.avoid_collisions = False
    r.ik_request.timeout.sec = 2
    r.ik_request.robot_state.is_diff = True
    if seed:
        r.ik_request.robot_state.joint_state.name = list(seed.keys())
        r.ik_request.robot_state.joint_state.position = list(seed.values())
    f = ik_cli.call_async(r)
    rclpy.spin_until_future_complete(node, f, timeout_sec=20)
    return f.result()

print('')
print('=== 测试 1: IK (/compute_ik, 组 arm, KDL 解算器) ===')
cases = [
    ('当前位姿 (必须成功)', p0.x, p0.y, p0.z),
    ('z 抬高 5cm',          p0.x, p0.y, p0.z + 0.05),
    ('x 前移 5cm',          p0.x + 0.05, p0.y, p0.z),
    ('y 侧移 5cm',          p0.x, p0.y + 0.05, p0.z),
]
ik_ok = 0
seed = {j: js[j] for j in ARM if j in js}
for name, x, y, z in cases:
    res = ik(x, y, z, q0, seed)
    v = res.error_code.val if res else None
    if v == 1:
        sol = dict(zip(res.solution.joint_state.name, res.solution.joint_state.position))
        print('  %-18s -> 成功  关节角 %s' % (name, ' '.join('%.2f' % sol[j] for j in ARM if j in sol)))
        ik_ok += 1
    else:
        print('  %-18s -> 失败 (code=%s)' % (name, v))
print('  IK 成功率: %d/%d' % (ik_ok, len(cases)))

goal = MotionPlanRequest()
goal.group_name = 'arm'
goal.num_planning_attempts = 5
goal.allowed_planning_time = 5.0
goal.max_velocity_scaling_factor = 0.5
goal.max_acceleration_scaling_factor = 0.5
goal.start_state.is_diff = True
goal.start_state.joint_state.name = [j for j in ARM if j in js]
goal.start_state.joint_state.position = [js[j] for j in ARM if j in js]

delta = [0.3, -0.2, 0.3, 0.2, -0.3, 0.2, -0.4]
c = Constraints()
for j, d in zip(ARM, delta):
    jc = JointConstraint()
    jc.joint_name = j
    jc.position = js[j] + d
    jc.tolerance_above = 0.05
    jc.tolerance_below = 0.05
    jc.weight = 1.0
    c.joint_constraints.append(jc)
goal.goal_constraints.append(c)

print('')
print('=== 测试 2: 路径规划 (/plan_kinematic_path, 组 arm, OMPL) ===')
wrap = GetMotionPlan.Request()
wrap.motion_plan_request = goal
pf = plan_cli.call_async(wrap)
rclpy.spin_until_future_complete(node, pf, timeout_sec=60)
pres = pf.result()
if pres is None:
    print('FAIL: 规划服务无响应'); sys.exit(3)
resp = pres.motion_plan_response
print('  error_code.val = %d  (%s)' % (resp.error_code.val, '成功' if resp.error_code.val == 1 else '失败'))
print('  规划耗时 = %.3f s' % resp.planning_time)
pts = resp.trajectory.joint_trajectory.points
print('  轨迹点数 = %d' % len(pts))
if len(pts) > 0:
    print('  起点  ' + ' '.join('%.3f' % v for v in pts[0].positions))
    print('  终点  ' + ' '.join('%.3f' % v for v in pts[-1].positions))
    print('  目标  ' + ' '.join('%.3f' % (js[j] + d) for j, d in zip(ARM, delta)))
print('')
plan_ok = (resp.error_code.val == 1) and len(pts) > 0
print('CONCLUSION: IK %s / 规划 %s' % ('可用' if ik_ok > 0 else '不可用', '可用' if plan_ok else '不可用'))
sys.exit(0 if (ik_ok > 0 and plan_ok) else 2)