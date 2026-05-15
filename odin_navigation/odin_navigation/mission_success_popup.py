import os
import threading
from typing import Optional

import rclpy
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from std_msgs.msg import String


class MissionSuccessPopup(Node):
    """Show a desktop popup when the mission success topic is published."""

    def __init__(self) -> None:
        super().__init__('mission_success_popup')

        self.declare_parameter('mission_status_topic', '/mission_status')
        self.declare_parameter('window_title', 'ODIN Mission Status')
        self.declare_parameter('message', 'MISSION SUCCESS')
        self.declare_parameter('subtitle', 'HOSTAGE RESCUE COMPLETE')

        self.window_title = str(self.get_parameter('window_title').value)
        self.message = str(self.get_parameter('message').value)
        self.subtitle = str(self.get_parameter('subtitle').value)
        self.popup_started = False
        self.tk_available = bool(os.environ.get('DISPLAY'))

        if not self.tk_available:
            self.get_logger().warning(
                'DISPLAY is not set; mission success popup is disabled for this session.'
            )

        self.create_subscription(
            String,
            str(self.get_parameter('mission_status_topic').value),
            self._status_callback,
            10,
        )
        self.get_logger().info('Mission success popup node ready.')

    def _status_callback(self, msg: String) -> None:
        if self.popup_started or msg.data != 'mission_success':
            return
        self.popup_started = True
        if not self.tk_available:
            self.get_logger().info('Mission success received, but popup is unavailable.')
            return

        thread = threading.Thread(target=self._show_popup, daemon=True)
        thread.start()

    def _show_popup(self) -> None:
        try:
            import tkinter as tk
        except ImportError:
            self.get_logger().warning(
                'python3-tk is not installed; cannot show mission success popup.'
            )
            return

        root = tk.Tk()
        root.title(self.window_title)
        root.configure(bg='#07130d')
        root.geometry('720x360')
        root.attributes('-topmost', True)

        container = tk.Frame(root, bg='#07130d', padx=40, pady=36)
        container.pack(expand=True, fill='both')

        title = tk.Label(
            container,
            text=self.message,
            fg='#39ff88',
            bg='#07130d',
            font=('DejaVu Sans', 42, 'bold'),
        )
        title.pack(pady=(40, 12))

        subtitle = tk.Label(
            container,
            text=self.subtitle,
            fg='#d9ffe8',
            bg='#07130d',
            font=('DejaVu Sans', 22, 'bold'),
        )
        subtitle.pack(pady=(0, 28))

        button = tk.Button(
            container,
            text='Close',
            command=root.destroy,
            font=('DejaVu Sans', 16),
            padx=24,
            pady=8,
        )
        button.pack()

        root.mainloop()


def main(args: Optional[list] = None) -> None:
    rclpy.init(args=args)
    node = MissionSuccessPopup()
    try:
        rclpy.spin(node)
    except (ExternalShutdownException, KeyboardInterrupt):
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
