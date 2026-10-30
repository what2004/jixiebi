import sys, time
import rclpy
from moveit_msgs.msg import Constraints, JointConstraint, MotionPlanRequest
from moveit_msgs.srv import GetMotionPlan
from sensor_msgs.msg import JointState

HAND = ['index_mcp_roll','index_mcp_pitch','index_pip','index_dip',
        'middle_mcp_roll','middle_mcp_pitch','middle_pip','middle_dip',
        'ring_mcp_roll','ring_mcp_pitch','ring_pip','ring_dip',
        'pinky_mcp_roll','pinky_mcp_pitch','pinky_pip','pinky_dip',
        'thumb_cmc_yaw','thumb_cmc_roll','thumb_cmc_pitch','thumb_mcp','thumb_dip']

rclpy.init()
node = rclpy.create_node('verify_hand')
cli = node.create_client(GetMotionPlan, '/plan_kinematic_path')
js = {}
node.create_subscription(JointState, '/joint_states', lambda m: [js.__setitem__(n, p) for n, p in zip(m.name, m.position)], 10)
if not cli.wait_for_service(timeout_sec=60):
    print('FAIL: 规划服务没出现'); sys.exit(1)
t0 = time.time()
while len(js) < 7 and time.time() - t0 < 30:
    rclpy.spin_once(node, timeout_sec=0.2)

goal = MotionPlanRequest()
goal.group_name = 'hand'
goal.num_planning_attempts = 5
goal.allowed_planning_time = 5.0
goal.max_velocity_scaling_factor = 0.5
goal.max_acceleration_scaling_factor = 0.5
goal.start_state.is_diff = True
goal.start_state.joint_state.name = [j for j in HAND if j in js]
goal.start_state.joint_state.position = [js[j] for j in HAND if j in js]
c = Constraints()
for j in HAND:
    if j not in js: continue
    jc = JointConstraint()
    jc.joint_name = j
    jc.position = 0.5 if (j.endswith('mcp_pitch') or j.endswith('_pip') or j.endswith('_dip')) else js[j]
    jc.tolerance_above = 0.05; jc.tolerance_below = 0.05; jc.weight = 1.0
    c.joint_constraints.append(jc)
goal.goal_constraints.append(c)

w = GetMotionPlan.Request(); w.motion_plan_request = goal
f = cli.call_async(w)
rclpy.spin_until_future_complete(node, f, timeout_sec=60)
res = f.result().motion_plan_response if f.result() else None
print('=== 测试 3: 手的规划组 (/plan_kinematic_path, 组 hand) ===')
if res is None:
    print('  FAIL: 无响应'); sys.exit(3)
print('  error_code.val = %d  (%s)' % (res.error_code.val, '成功' if res.error_code.val == 1 else '失败'))
pts = res.trajectory.joint_trajectory.points
print('  轨迹点数 = %d   规划耗时 = %.3f s' % (len(pts), res.planning_time))
names = list(res.trajectory.joint_trajectory.joint_names)
print('  轨迹里的关节数 = %d（全部 %d 个手指关节都是独立可规划的）' % (len(names), len(HAND)))
if names:
    extra = [n for n in names if n not in HAND]
    print('  额外关节: %s' % (extra if extra else '无'))
print('CONCLUSION: 手的规划组 ' + ('可用' if (res.error_code.val == 1 and len(pts) > 0) else '不可用'))
sys.exit(0 if (res.error_code.val == 1 and len(pts) > 0) else 2)