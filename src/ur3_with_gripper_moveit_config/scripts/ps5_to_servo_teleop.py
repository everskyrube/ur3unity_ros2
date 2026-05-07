#!/usr/bin/env python3
"""PS5 DualSense dual-mode teleop: Driving (UGV) + Manipulator (arm servo).

R1 toggles between DRIVING and MANIPULATOR mode (rising edge).

DRIVING mode:
  Left stick Y  -> linear.x (forward / backward)
  Right stick X -> angular.z (turn left / right)

MANIPULATOR mode:
  L1 held -> joint-jog; released -> Cartesian
  Cartesian:
    Left stick X/Y  -> XY in base_link
    Right stick X/Y -> roll / pitch
    D-pad up/down   -> Z up/down
    Triangle/Square -> yaw +/-
  Joint:
    Left stick X/Y  -> shoulder_pan / shoulder_lift
    Right stick X/Y -> wrist_1 / elbow
    D-pad X/Y      -> wrist_2 / wrist_3

Gripper (any mode): Cross -> open, Circle -> close
Mode published on /teleop/mode (std_msgs/String).
"""
import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, QoSReliabilityPolicy, QoSHistoryPolicy
from enum import Enum

from sensor_msgs.msg import Joy, JointState
from geometry_msgs.msg import Twist, TwistStamped
from control_msgs.msg import JointJog
from std_msgs.msg import String
from std_srvs.srv import Trigger


class Mode(Enum):
    DRIVING = 0
    MANIPULATOR = 1


ARM_JOINTS = [
    "shoulder_pan_joint",
    "shoulder_lift_joint",
    "elbow_joint",
    "wrist_1_joint",
    "wrist_2_joint",
    "wrist_3_joint",
]


class Ps5DualModeTeleop(Node):
    def __init__(self):
        super().__init__("ps5_to_servo_teleop")

        p = self.declare_parameter
        p("deadzone", 0.12)

        # Arm scales
        p("linear_scale", 0.9)
        p("angular_scale", 0.9)
        p("joint_scale", 0.9)
        p("base_frame", "base_link")
        p("ee_frame", "tool0")

        # Driving scales (Jackal UGV)
        p("drive_linear_scale", 0.5)
        p("drive_angular_scale", 1.0)

        # Gripper
        p("gripper_min", 0.0)
        p("gripper_max", 0.695)
        p("gripper_step", 0.02)
        p("gripper_rate_hz", 50.0)

        # Axes
        p("axis_left_x", 0)
        p("axis_left_y", 1)
        p("axis_l2", 2)
        p("axis_right_x", 3)
        p("axis_right_y", 4)
        p("axis_r2", 5)
        p("axis_dpad_x", 6)
        p("axis_dpad_y", 7)

        # Buttons
        p("btn_cross", 0)
        p("btn_circle", 1)
        p("btn_square", 2)
        p("btn_triangle", 3)
        p("btn_l1", 4)
        p("btn_r1", 5)
        p("btn_l2", 6)
        p("btn_r2", 7)
        p("btn_dpad_up", 11)
        p("btn_dpad_down", 12)

        # Watchdog
        p("watchdog_timeout", 0.3)

        qos = QoSProfile(
            reliability=QoSReliabilityPolicy.RELIABLE,
            history=QoSHistoryPolicy.KEEP_LAST,
            depth=10,
        )

        # Arm publishers
        self.twist_pub = self.create_publisher(
            TwistStamped, "/servo_node/delta_twist_cmds", qos)
        self.joint_pub = self.create_publisher(
            JointJog, "/servo_node/delta_joint_cmds", qos)
        self.gripper_state_pub = self.create_publisher(
            JointState, "/gripper/joint_states", qos)

        # Driving publisher
        self.cmd_vel_pub = self.create_publisher(Twist, "/cmd_vel", qos)

        # Mode publisher
        self.mode_pub = self.create_publisher(String, "/teleop/mode", qos)

        # Joy subscriber
        self.joy_sub = self.create_subscription(Joy, "/joy", self._on_joy, qos)

        # Servo startup
        self._servo_start = self.create_client(Trigger, "/servo_node/start_servo")
        self._servo_started = False
        self.create_timer(1.0, self._try_start_servo)

        # Gripper state
        self._gripper_pos = 0.0
        self._gripper_open_held = False
        self._gripper_close_held = False
        rate = float(self.get_parameter("gripper_rate_hz").value)
        self.create_timer(1.0 / max(rate, 1.0), self._tick_gripper)

        # Mode toggle state
        self._mode = Mode.DRIVING
        self._prev_r1 = False

        # Watchdog
        self._last_joy_stamp = None
        timeout = float(self.get_parameter("watchdog_timeout").value)
        self.create_timer(timeout / 2.0, self._watchdog_tick)

        self._publish_mode()

        self.get_logger().info(
            "Dual-mode teleop ready. Press R1 to toggle DRIVING <-> MANIPULATOR. "
            "Starting in DRIVING mode.")

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

    def _publish_mode(self):
        msg = String()
        msg.data = self._mode.name
        self.mode_pub.publish(msg)

    def _zero_driving(self):
        self.cmd_vel_pub.publish(Twist())

    def _zero_arm(self):
        t = TwistStamped()
        t.header.stamp = self.get_clock().now().to_msg()
        t.header.frame_id = str(self._p("base_frame"))
        self.twist_pub.publish(t)

    def _watchdog_tick(self):
        if self._last_joy_stamp is None:
            return
        timeout = float(self._p("watchdog_timeout"))
        elapsed = (self.get_clock().now() - self._last_joy_stamp).nanoseconds / 1e9
        if elapsed > timeout:
            self._zero_driving()
            self._zero_arm()

    def _on_joy(self, msg: Joy):
        self._last_joy_stamp = self.get_clock().now()

        # Gripper buttons work in any mode
        self._gripper_open_held = self._btn(msg, int(self._p("btn_cross")))
        self._gripper_close_held = self._btn(msg, int(self._p("btn_circle")))

        # R1 toggle (rising edge only)
        r1 = self._btn(msg, int(self._p("btn_r1")))
        if r1 and not self._prev_r1:
            old_mode = self._mode
            if self._mode == Mode.DRIVING:
                self._zero_driving()
                self._mode = Mode.MANIPULATOR
            else:
                self._zero_arm()
                self._mode = Mode.DRIVING
            self._publish_mode()
            self.get_logger().info(f"Mode: {old_mode.name} -> {self._mode.name}")
        self._prev_r1 = r1

        # Dispatch
        if self._mode == Mode.DRIVING:
            self._publish_driving(msg)
        else:
            joint_mode = self._btn(msg, int(self._p("btn_l1")))
            if joint_mode:
                self._publish_joint(msg)
            else:
                self._publish_twist(msg)

    def _publish_driving(self, msg: Joy):
        lin_scale = float(self._p("drive_linear_scale"))
        ang_scale = float(self._p("drive_angular_scale"))

        ly = self._deadzone(self._axis(msg, int(self._p("axis_left_y"))))
        rx = self._deadzone(self._axis(msg, int(self._p("axis_right_x"))))

        twist = Twist()
        twist.linear.x = lin_scale * ly
        twist.angular.z = ang_scale * rx
        self.cmd_vel_pub.publish(twist)

    def _publish_twist(self, msg: Joy):
        lin = float(self._p("linear_scale"))
        ang = float(self._p("angular_scale"))
        lx = self._deadzone(self._axis(msg, int(self._p("axis_left_x"))))
        ly = self._deadzone(self._axis(msg, int(self._p("axis_left_y"))))
        rx = self._deadzone(self._axis(msg, int(self._p("axis_right_x"))))
        ry = self._deadzone(self._axis(msg, int(self._p("axis_right_y"))))

        z_up = self._btn(msg, int(self._p("btn_dpad_up")))
        z_down = self._btn(msg, int(self._p("btn_dpad_down")))
        z = float(z_up) - float(z_down)

        yaw_pos = self._btn(msg, int(self._p("btn_triangle")))
        yaw_neg = self._btn(msg, int(self._p("btn_square")))
        yaw = float(yaw_pos) - float(yaw_neg)

        twist = TwistStamped()
        twist.header.stamp = self.get_clock().now().to_msg()
        twist.header.frame_id = str(self._p("base_frame"))
        twist.twist.linear.x = lin * ly
        twist.twist.linear.y = lin * -lx
        twist.twist.linear.z = lin * z
        twist.twist.angular.x = ang * ry
        twist.twist.angular.y = ang * -rx
        twist.twist.angular.z = ang * yaw
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
        js = JointState()
        js.header.stamp = self.get_clock().now().to_msg()
        js.name = ["finger_joint"]
        js.position = [self._gripper_pos]
        self.gripper_state_pub.publish(js)


def main():
    rclpy.init()
    node = Ps5DualModeTeleop()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
