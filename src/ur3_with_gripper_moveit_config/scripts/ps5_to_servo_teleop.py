#!/usr/bin/env python3
"""PS5 DualSense → moveit_servo teleop bridge.

Control layout (DualSense, hid-playstation kernel driver):
  R1  (hold)           — deadman: arm only moves while held
  L1  (hold + R1)      — joint-jog mode; release = Cartesian mode
  Cartesian mode (default):
    Left stick X/Y     — planar XY in base_link
    Right stick X/Y    — roll / pitch
    L2 / R2 triggers   — Z down / Z up
  Joint mode (L1 + R1):
    Left stick X/Y     — shoulder_pan / shoulder_lift
    Right stick X/Y    — wrist_1 / elbow
    D-pad X / Y        — wrist_2 / wrist_3
  Gripper (no deadman needed):
    X  (cross)         — open  (decrease finger_joint)
    O  (circle)        — close (increase finger_joint)
"""
import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, QoSReliabilityPolicy, QoSHistoryPolicy

from sensor_msgs.msg import Joy, JointState
from geometry_msgs.msg import TwistStamped
from control_msgs.msg import JointJog
from std_srvs.srv import Trigger


ARM_JOINTS = [
    "shoulder_pan_joint",
    "shoulder_lift_joint",
    "elbow_joint",
    "wrist_1_joint",
    "wrist_2_joint",
    "wrist_3_joint",
]


class Ps5ToServoTeleop(Node):
    def __init__(self):
        super().__init__("ps5_to_servo_teleop")

        p = self.declare_parameter
        p("deadzone", 0.12)
        p("linear_scale", 0.9)
        p("angular_scale", 0.9)
        p("joint_scale", 0.9)
        p("base_frame", "base_link")
        p("ee_frame", "tool0")
        p("gripper_min", 0.0)
        p("gripper_max", 0.695)
        p("gripper_step", 0.02)
        p("gripper_rate_hz", 50.0)
        p("axis_left_x", 0);  p("axis_left_y", 1);  p("axis_l2", 2)
        p("axis_right_x", 3); p("axis_right_y", 4); p("axis_r2", 5)
        p("axis_dpad_x", 6);  p("axis_dpad_y", 7)
        p("btn_cross", 0);    p("btn_circle", 1)
        p("btn_square", 2);   p("btn_triangle", 3)
        p("btn_l1", 4);       p("btn_r1", 5)
        p("btn_l2", 6);       p("btn_r2", 7)

        qos = QoSProfile(
            reliability=QoSReliabilityPolicy.RELIABLE,
            history=QoSHistoryPolicy.KEEP_LAST,
            depth=10,
        )
        self.twist_pub = self.create_publisher(
            TwistStamped, "/servo_node/delta_twist_cmds", qos)
        self.joint_pub = self.create_publisher(
            JointJog, "/servo_node/delta_joint_cmds", qos)
        #self.gripper_state_pub = self.create_publisher(
        #    JointState, "/joint_states", qos)

        # CHANGED: Publish to /gripper/joint_states instead of /joint_states
        self.gripper_state_pub = self.create_publisher(
            JointState, "/gripper/joint_states", qos)
            
        self.joy_sub = self.create_subscription(Joy, "/joy", self._on_joy, qos)

        self._servo_start = self.create_client(Trigger, "/servo_node/start_servo")
        self._servo_started = False
        self.create_timer(1.0, self._try_start_servo)

        self._gripper_pos = 0.0
        self._gripper_open_held = False
        self._gripper_close_held = False
        rate = float(self.get_parameter("gripper_rate_hz").value)
        self.create_timer(1.0 / max(rate, 1.0), self._tick_gripper)

        self.get_logger().info(
            "PS5 teleop ready. Hold R1 to move. Hold L1+R1 for joint mode. "
            "Cross = open gripper, Circle = close gripper.")

    def _try_start_servo(self):
        if self._servo_started:
            return
        if not self._servo_start.service_is_ready():
            return
        future = self._servo_start.call_async(Trigger.Request())
        future.add_done_callback(self._on_servo_started)

    def _on_servo_started(self, future):
        try:
            resp = future.result()
        except Exception as exc:
            self.get_logger().warn(f"start_servo call failed: {exc}")
            return
        if resp.success:
            self._servo_started = True
            self.get_logger().info("servo_node started.")
        else:
            self.get_logger().warn(f"start_servo returned: {resp.message}")

    def _p(self, name):
        return self.get_parameter(name).value

    def _axis(self, msg: Joy, idx: int) -> float:
        return msg.axes[idx] if 0 <= idx < len(msg.axes) else 0.0

    def _btn(self, msg: Joy, idx: int) -> bool:
        return bool(msg.buttons[idx]) if 0 <= idx < len(msg.buttons) else False

    def _deadzone(self, v: float) -> float:
        dz = float(self._p("deadzone"))
        if abs(v) < dz:
            return 0.0
        sign = 1.0 if v > 0 else -1.0
        return (v - sign * dz) / (1.0 - dz)

    def _trigger_01(self, raw: float) -> float:
        # DualSense triggers rest at +1, become -1 fully pressed. Normalize to [0, 1].
        return max(0.0, min(1.0, (1.0 - raw) * 0.5))

    def _on_joy(self, msg: Joy):
        # Gripper buttons — latch level, released in tick if lifted
        self._gripper_open_held  = self._btn(msg, int(self._p("btn_cross")))
        self._gripper_close_held = self._btn(msg, int(self._p("btn_circle")))

        # Deadman
        if not self._btn(msg, int(self._p("btn_r1"))):
            return

        joint_mode = self._btn(msg, int(self._p("btn_l1")))
        if joint_mode:
            self._publish_joint(msg)
        else:
            self._publish_twist(msg)

    def _publish_twist(self, msg: Joy):
        lin = float(self._p("linear_scale"))
        ang = float(self._p("angular_scale"))
        lx = self._deadzone(self._axis(msg, int(self._p("axis_left_x"))))
        ly = self._deadzone(self._axis(msg, int(self._p("axis_left_y"))))
        rx = self._deadzone(self._axis(msg, int(self._p("axis_right_x"))))
        ry = self._deadzone(self._axis(msg, int(self._p("axis_right_y"))))
        l2 = self._trigger_01(self._axis(msg, int(self._p("axis_l2"))))
        r2 = self._trigger_01(self._axis(msg, int(self._p("axis_r2"))))

        twist = TwistStamped()
        twist.header.stamp = self.get_clock().now().to_msg()
        twist.header.frame_id = str(self._p("base_frame"))
        twist.twist.linear.x = lin * ly
        twist.twist.linear.y = lin * -lx
        twist.twist.linear.z = lin * (r2 - l2)
        twist.twist.angular.x = ang * ry
        twist.twist.angular.y = ang * -rx
        twist.twist.angular.z = 0.0
        self.twist_pub.publish(twist)

    def _publish_joint(self, msg: Joy):
        s = float(self._p("joint_scale"))
        lx = self._deadzone(self._axis(msg, int(self._p("axis_left_x"))))
        ly = self._deadzone(self._axis(msg, int(self._p("axis_left_y"))))
        rx = self._deadzone(self._axis(msg, int(self._p("axis_right_x"))))
        ry = self._deadzone(self._axis(msg, int(self._p("axis_right_y"))))
        dx = self._axis(msg, int(self._p("axis_dpad_x")))
        dy = self._axis(msg, int(self._p("axis_dpad_y")))

        jog = JointJog()
        jog.header.stamp = self.get_clock().now().to_msg()
        jog.joint_names = list(ARM_JOINTS)
        jog.velocities = [
            s * lx,   # shoulder_pan
            s * -ly,  # shoulder_lift
            s * -ry,  # elbow
            s * rx,   # wrist_1
            s * dx,   # wrist_2
            s * dy,   # wrist_3
        ]
        self.joint_pub.publish(jog)

    def _tick_gripper(self):
        step = float(self._p("gripper_step"))
        lo = float(self._p("gripper_min"))
        hi = float(self._p("gripper_max"))
        if self._gripper_open_held and not self._gripper_close_held:
            self._gripper_pos = max(lo, self._gripper_pos - step)
        elif self._gripper_close_held and not self._gripper_open_held:
            self._gripper_pos = min(hi, self._gripper_pos + step)
        # Always publish current position so robot_state_publisher has TFs for the
        # mimic-driven gripper chain even while no button is pressed.
        js = JointState()
        js.header.stamp = self.get_clock().now().to_msg()
        js.name = ["finger_joint"]
        js.position = [self._gripper_pos]
        self.gripper_state_pub.publish(js)


def main():
    rclpy.init()
    node = Ps5ToServoTeleop()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
