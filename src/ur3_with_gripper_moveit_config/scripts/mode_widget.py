#!/usr/bin/env python3
"""Mode display widget for PS5 dual-mode teleop.

Subscribes to /teleop/mode and shows the current control mode
in a small always-on-top tkinter window with a toggle indicator.

Run standalone:  python3 mode_widget.py
Or via ROS:      ros2 run ur3_with_gripper_moveit_config mode_widget.py
"""
import threading
import tkinter as tk

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, QoSReliabilityPolicy, QoSHistoryPolicy
from std_msgs.msg import String


MODES = {
    "DRIVING":      {"bg": "#27ae60", "fg": "#ffffff", "side": "left"},
    "MANIPULATOR":  {"bg": "#c0392b", "fg": "#ffffff", "side": "right"},
}
WAITING_BG = "#7f8c8d"


class ModeWidgetNode(Node):
    def __init__(self, callback):
        super().__init__("mode_widget")
        qos = QoSProfile(
            reliability=QoSReliabilityPolicy.RELIABLE,
            history=QoSHistoryPolicy.KEEP_LAST,
            depth=10,
        )
        self._cb = callback
        self.create_subscription(String, "/teleop/mode", self._on_mode, qos)
        self.get_logger().info("Mode widget listening on /teleop/mode")

    def _on_mode(self, msg):
        self._cb(msg.data)


class ModeWidget:
    TOGGLE_W = 60
    TOGGLE_H = 28
    KNOB_R = 10

    def __init__(self):
        self._root = tk.Tk()
        self._root.title("Teleop Mode")
        self._root.attributes("-topmost", True)
        self._root.resizable(False, False)
        self._root.configure(bg=WAITING_BG)

        # Mode label
        self._label = tk.Label(
            self._root,
            text="WAITING...",
            font=("Helvetica", 24, "bold"),
            fg="#ffffff",
            bg=WAITING_BG,
            padx=20,
            pady=10,
        )
        self._label.pack()

        # Toggle switch canvas
        self._canvas = tk.Canvas(
            self._root,
            width=self.TOGGLE_W,
            height=self.TOGGLE_H,
            bg=WAITING_BG,
            highlightthickness=0,
        )
        self._canvas.pack(pady=(0, 8))

        pad = 2
        self._track = self._canvas.create_rectangle(
            pad, pad, self.TOGGLE_W - pad, self.TOGGLE_H - pad,
            fill="#555555", outline="", width=0,
        )
        cx = pad + self.KNOB_R + 2
        cy = self.TOGGLE_H // 2
        self._knob = self._canvas.create_oval(
            cx - self.KNOB_R, cy - self.KNOB_R,
            cx + self.KNOB_R, cy + self.KNOB_R,
            fill="#ffffff", outline="",
        )

        # Sub-labels for the two sides
        self._sub = tk.Label(
            self._root,
            text="DRIVE        ARM",
            font=("Helvetica", 9),
            fg="#cccccc",
            bg=WAITING_BG,
        )
        self._sub.pack(pady=(0, 6))

        self._current_mode = None
        self._pending_mode = None
        self._lock = threading.Lock()

        # ROS in background thread
        rclpy.init()
        self._node = ModeWidgetNode(self._on_mode_received)
        self._spin_thread = threading.Thread(target=self._spin_ros, daemon=True)
        self._spin_thread.start()

        self._root.after(50, self._poll_mode)
        self._root.protocol("WM_DELETE_WINDOW", self._on_close)

    def _spin_ros(self):
        try:
            rclpy.spin(self._node)
        except Exception:
            pass

    def _on_mode_received(self, mode_str):
        with self._lock:
            self._pending_mode = mode_str

    def _poll_mode(self):
        with self._lock:
            mode = self._pending_mode

        if mode is not None and mode != self._current_mode:
            self._current_mode = mode
            info = MODES.get(mode, {"bg": WAITING_BG, "fg": "#ffffff", "side": "left"})
            bg, fg = info["bg"], info["fg"]

            self._label.config(text=mode, bg=bg, fg=fg)
            self._root.config(bg=bg)
            self._canvas.config(bg=bg)
            self._sub.config(bg=bg)
            self._canvas.itemconfig(self._track, fill=self._darken(bg))

            # Move knob
            pad = 2
            cy = self.TOGGLE_H // 2
            if info["side"] == "left":
                cx = pad + self.KNOB_R + 2
            else:
                cx = self.TOGGLE_W - pad - self.KNOB_R - 2
            self._canvas.coords(
                self._knob,
                cx - self.KNOB_R, cy - self.KNOB_R,
                cx + self.KNOB_R, cy + self.KNOB_R,
            )

        self._root.after(50, self._poll_mode)

    @staticmethod
    def _darken(hex_color, factor=0.7):
        r = int(hex_color[1:3], 16)
        g = int(hex_color[3:5], 16)
        b = int(hex_color[5:7], 16)
        return f"#{int(r*factor):02x}{int(g*factor):02x}{int(b*factor):02x}"

    def _on_close(self):
        self._node.destroy_node()
        rclpy.shutdown()
        self._root.destroy()

    def run(self):
        self._root.mainloop()


def main():
    widget = ModeWidget()
    widget.run()


if __name__ == "__main__":
    main()
