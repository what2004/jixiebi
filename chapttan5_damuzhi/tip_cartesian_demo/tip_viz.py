#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
tip_viz.py —— RViz 可视化 + 实际轨迹采集（ROS 相关，只干这一件事）

发一个 MarkerArray 到 /tip_cartesian_demo/trace：
  绿线 = 理想直线  绿球 = 终点
  蓝点 = 规划出的路径点
  红线 = 实际指尖轨迹（执行时从 /joint_states 实时累积）
RViz 里 Add -> By topic -> /tip_cartesian_demo/trace -> MarkerArray 即可看到。
"""
import numpy as np

from tip_core import JOINTS, fk_tip


class TipTraceNode:
    TOPIC = '/tip_cartesian_demo/trace'

    def __init__(self, node, p0=None, p1=None, planned=None, publish=False, frame='base_link'):
        self.node = node
        self.frame = frame
        self.p0 = None if p0 is None else np.asarray(p0, float)
        self.p1 = None if p1 is None else np.asarray(p1, float)
        if planned is None:                 # 注意：不要用 "planned or []"（numpy 真值会报错）
            planned = []
        self.planned = [np.asarray(q, float) for q in planned]
        self.trace = []
        self.joint_trace = []      # (t, q[5], v[5]|None)：执行期间 /joint_states 采样，供 --actual-csv 导出
        self.pub = None

        from sensor_msgs.msg import JointState
        from geometry_msgs.msg import PointStamped
        from std_msgs.msg import Float64MultiArray
        # 无论是否发 Marker 都订阅关节角：以便事后校验"实际走直了没有"
        node.create_subscription(JointState, '/joint_states', self._on_joint_state, 10)
        # 给 PlotJuggler 用的实时曲线：实际指尖位置 + 沿直线进度/垂距
        self.pub_tip = node.create_publisher(PointStamped, '/tip_cartesian_demo/tip_point', 10)
        self.pub_metric = node.create_publisher(Float64MultiArray, '/tip_cartesian_demo/tip_metric', 10)
        if publish:
            from visualization_msgs.msg import MarkerArray
            self.pub = node.create_publisher(MarkerArray, self.TOPIC, 10)
            node.create_timer(0.05, self._publish)
            print('RViz: 加一个 MarkerArray 显示, topic = %s' % self.TOPIC)
            print('      绿线=理想直线  蓝点=规划路径点  红线=实际指尖轨迹(实时)')

    # ---------- 采集实际指尖位置 ----------
    def _on_joint_state(self, msg):
        d = dict(zip(msg.name, msg.position))
        if not all(j in d for j in JOINTS):
            return
        q = np.array([d[j] for j in JOINTS])
        p = fk_tip(q)
        self._publish_live(p)
        if not self.trace or np.linalg.norm(p - self.trace[-1]) > 2e-4:
            self.trace.append(p)
            if len(self.trace) > 5000:
                self.trace = self.trace[-5000:]
        # 关节数据也记一份（带时间戳/速度），供 --actual-csv 导出
        st = msg.header.stamp
        if st.sec or st.nanosec:
            t = st.sec + st.nanosec * 1e-9
        else:
            now = self.node.get_clock().now().to_msg()
            t = now.sec + now.nanosec * 1e-9
        v = None
        if len(msg.velocity) == len(msg.name):
            dv = dict(zip(msg.name, msg.velocity))
            if all(j in dv for j in JOINTS):
                v = np.array([dv[j] for j in JOINTS])
        if not self.joint_trace or np.max(np.abs(q - self.joint_trace[-1][1])) > 1e-6:
            self.joint_trace.append((t, q, v))
            if len(self.joint_trace) > 20000:
                self.joint_trace = self.joint_trace[-20000:]

    # ---------- 给 PlotJuggler 的实时数据 ----------
    def _publish_live(self, p):
        from geometry_msgs.msg import PointStamped
        from std_msgs.msg import Float64MultiArray
        msg = PointStamped()
        msg.header.frame_id = self.frame
        msg.header.stamp = self.node.get_clock().now().to_msg()
        msg.point.x, msg.point.y, msg.point.z = float(p[0]), float(p[1]), float(p[2])
        self.pub_tip.publish(msg)

        m = Float64MultiArray()
        if self.p0 is not None and self.p1 is not None:
            line = self.p1 - self.p0
            L = float(np.linalg.norm(line))
            u = line / L if L > 1e-9 else line
            v = p - self.p0
            m.data = [float(np.dot(v, u)) * 1000.0,                       # data[0] 沿直线走过的距离 mm
                      float(np.linalg.norm(v - np.dot(v, u) * u)) * 1000.0,  # data[1] 到理想直线垂距 mm
                      L * 1000.0]                                          # data[2] 目标长度 mm
        else:
            m.data = [float('nan')] * 3
        self.pub_metric.publish(m)

    # ---------- 发 Marker ----------
    def _marker(self, mid, mtype, rgba, scale):
        from visualization_msgs.msg import Marker
        m = Marker()
        m.header.frame_id = self.frame
        m.header.stamp = self.node.get_clock().now().to_msg()
        m.ns = 'tip_cartesian_demo'
        m.id = mid
        m.type = mtype
        m.action = Marker.ADD
        m.pose.orientation.w = 1.0
        m.color.r, m.color.g, m.color.b, m.color.a = rgba
        m.scale.x = m.scale.y = m.scale.z = scale
        m.lifetime.sec = 0            # 永久，跑完画面仍保留
        return m

    @staticmethod
    def _pt(p):
        from geometry_msgs.msg import Point
        q = Point()
        q.x, q.y, q.z = float(p[0]), float(p[1]), float(p[2])
        return q

    def _publish(self):
        if self.pub is None:
            return
        from visualization_msgs.msg import Marker, MarkerArray
        arr = MarkerArray()
        if self.p0 is not None and self.p1 is not None:
            m = self._marker(0, Marker.LINE_STRIP, (0.1, 1.0, 0.1, 1.0), 0.0015)
            m.points = [self._pt(self.p0), self._pt(self.p1)]
            arr.markers.append(m)
            m = self._marker(3, Marker.SPHERE, (0.1, 1.0, 0.1, 1.0), 0.005)
            m.pose.position = self._pt(self.p1)
            arr.markers.append(m)
        if self.planned:
            m = self._marker(1, Marker.SPHERE_LIST, (0.2, 0.4, 1.0, 1.0), 0.0012)
            m.points = [self._pt(q) for q in self.planned]
            arr.markers.append(m)
        m = self._marker(2, Marker.LINE_STRIP, (1.0, 0.15, 0.15, 1.0), 0.0018)
        m.points = [self._pt(q) for q in self.trace]
        arr.markers.append(m)
        self.pub.publish(arr)

    def write_joint_csv(self, path):
        """执行期间从 /joint_states 采到的关节数据 -> CSV（含正解指尖 xyz 与到直线垂距）

        列：t, q0..q4, x_m,y_m,z_m, dev_mm,（有速度时再补 v0..v4）
        """
        if not self.joint_trace:
            print('实际关节数据: 没采到点（执行期间没收到 /joint_states？）')
            return False
        has_v = any(s[2] is not None for s in self.joint_trace)
        with open(path, 'w') as f:
            if self.p0 is not None and self.p1 is not None:
                f.write('# P0_m=%.6f,%.6f,%.6f\n' % tuple(self.p0))
                f.write('# P1_m=%.6f,%.6f,%.6f\n' % tuple(self.p1))
            f.write('# joints=%s\n' % ','.join(JOINTS))
            hdr = 't,q0,q1,q2,q3,q4,x_m,y_m,z_m,dev_mm'
            if has_v:
                hdr += ',v0,v1,v2,v3,v4'
            f.write(hdr + '\n')
            base = np.asarray(self.p0, float) if self.p0 is not None else None
            endp = np.asarray(self.p1, float) if self.p1 is not None else None
            u, b0 = None, None
            if base is not None and endp is not None:
                line = endp - base
                L = float(np.linalg.norm(line))
                if L > 1e-9:
                    u = line / L
                    b0 = base
            for t, q, v in self.joint_trace:
                p = fk_tip(q)
                dev = 0.0
                if u is not None:
                    dv = p - b0
                    dev = float(np.linalg.norm(dv - np.dot(dv, u) * u)) * 1000.0
                row = '%.4f,%s,%.6f,%.6f,%.6f,%.5f' % (
                    t, ','.join('%.6f' % x for x in q), p[0], p[1], p[2], dev)
                if has_v:
                    vv = v if v is not None else [float('nan')] * 5
                    row += ',' + ','.join('%.5f' % x for x in vv)
                f.write(row + '\n')
        Q = np.array([s[1] for s in self.joint_trace])
        print('已导出实际执行关节数据: %s   (%d 个采样点, 来源 /joint_states)' % (path, len(self.joint_trace)))
        print('实际执行各关节变化 (rad)：')
        for i, j in enumerate(JOINTS):
            print('  %-16s min = %8.4f   max = %8.4f   变化幅度 = %.4f'
                  % (j, Q[:, i].min(), Q[:, i].max(), Q[:, i].max() - Q[:, i].min()))
        return True


def spin_for(node, seconds):
    """阻塞式地转 node 若干秒（用于保持 Marker 显示）"""
    import rclpy
    import time
    t0 = time.time()
    while rclpy.ok() and time.time() - t0 < seconds:
        rclpy.spin_once(node, timeout_sec=0.05)
