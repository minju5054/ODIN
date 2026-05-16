import os
import queue
import threading
from datetime import datetime
from typing import Optional

import rclpy
from odin_interfaces.msg import HostageEvent
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from std_msgs.msg import String


class QwenDialogPanel(Node):
    """Show coordinator and Qwen decision logs in a desktop popup."""

    def __init__(self) -> None:
        super().__init__('qwen_dialog_panel')

        self.declare_parameter('hostage_event_topic', '/hostage_events')
        self.declare_parameter('coordinator_status_topic', '/coordinator/status')
        self.declare_parameter('ai_status_topic', '/ai/status')
        self.declare_parameter('window_title', 'ODIN Coordinator / Qwen Dialog')
        self.declare_parameter('max_lines', 120)

        self.window_title = str(self.get_parameter('window_title').value)
        self.max_lines = int(self.get_parameter('max_lines').value)
        self.event_queue: queue.Queue = queue.Queue()
        self.tk_available = bool(os.environ.get('DISPLAY'))

        if self.tk_available:
            thread = threading.Thread(target=self._run_tk, daemon=True)
            thread.start()
        else:
            self.get_logger().warning(
                'DISPLAY is not set; Qwen dialog panel is disabled for this session.'
            )

        self.create_subscription(
            HostageEvent,
            str(self.get_parameter('hostage_event_topic').value),
            self._hostage_event_callback,
            10,
        )
        self.create_subscription(
            String,
            str(self.get_parameter('coordinator_status_topic').value),
            self._coordinator_status_callback,
            10,
        )
        self.create_subscription(
            String,
            str(self.get_parameter('ai_status_topic').value),
            self._ai_status_callback,
            10,
        )

        self.get_logger().info('Qwen dialog panel ready.')

    def _hostage_event_callback(self, msg: HostageEvent) -> None:
        self._enqueue(
            'SCOUT',
            (
                f'{self._format_robot_name(msg.detecting_robot)} detected hostage '
                f'id={msg.marker_id} at x={msg.pose.position.x:.2f}, y={msg.pose.position.y:.2f}'
            ),
        )

    def _coordinator_status_callback(self, msg: String) -> None:
        if msg.data == 'coordinator_ready':
            return
        label = 'COORDINATOR'
        if 'candidate_ready' in msg.data:
            label = 'COORDINATOR -> QWEN'
        elif 'ai_waypoint_validated' in msg.data:
            label = 'COORDINATOR -> ROBOT-3'
        self._enqueue(label, msg.data)

    def _ai_status_callback(self, msg: String) -> None:
        if msg.data.startswith('virtual_qwen_ready'):
            self._enqueue('QWEN', msg.data)
            return
        label = 'QWEN'
        if 'qwen_request_started' in msg.data:
            label = 'COORDINATOR -> QWEN'
        elif 'qwen_response_received' in msg.data:
            label = 'QWEN -> COORDINATOR'
        elif 'qwen_selected_route' in msg.data:
            label = 'QWEN DECISION'
        elif 'qwen_request_failed' in msg.data or 'fallback' in msg.data:
            label = 'QWEN FALLBACK'
        self._enqueue(label, msg.data)

    def _enqueue(self, source: str, text: str) -> None:
        timestamp = datetime.now().strftime('%H:%M:%S')
        self.event_queue.put((timestamp, source, text))

    @staticmethod
    def _format_robot_name(robot_name: str) -> str:
        if not robot_name:
            return 'ROBOT'
        return robot_name.replace('robot_', 'ROBOT-').replace('robot', 'ROBOT').upper()

    def _run_tk(self) -> None:
        try:
            import tkinter as tk
            from tkinter import scrolledtext
        except ImportError:
            self.get_logger().warning(
                'python3-tk is not installed; cannot show Qwen dialog panel.'
            )
            return

        root = tk.Tk()
        root.title(self.window_title)
        root.configure(bg='#071018')
        root.geometry('920x620')
        root.attributes('-topmost', True)

        container = tk.Frame(root, bg='#071018', padx=24, pady=20)
        container.pack(expand=True, fill='both')

        title = tk.Label(
            container,
            text='COORDINATOR / QWEN DIALOG',
            fg='#d7ecff',
            bg='#071018',
            font=('DejaVu Sans', 24, 'bold'),
        )
        title.pack(anchor='w', pady=(0, 6))

        subtitle = tk.Label(
            container,
            text='live route request, Qwen response, and dispatch decision log',
            fg='#8fa3b2',
            bg='#071018',
            font=('DejaVu Sans', 13, 'bold'),
        )
        subtitle.pack(anchor='w', pady=(0, 14))

        log = scrolledtext.ScrolledText(
            container,
            bg='#0c141d',
            fg='#d7ecff',
            insertbackground='#d7ecff',
            relief='flat',
            wrap='word',
            height=24,
            font=('DejaVu Sans Mono', 12),
        )
        log.pack(expand=True, fill='both')
        log.configure(state='disabled')
        log.tag_config('SCOUT', foreground='#82d8ff')
        log.tag_config('COORDINATOR', foreground='#ffd166')
        log.tag_config('COORDINATOR -> QWEN', foreground='#ffb86b')
        log.tag_config('QWEN', foreground='#b9fbc0')
        log.tag_config('QWEN -> COORDINATOR', foreground='#7cffc4')
        log.tag_config('QWEN DECISION', foreground='#00e5ff')
        log.tag_config('QWEN FALLBACK', foreground='#ff6b6b')
        log.tag_config('COORDINATOR -> ROBOT-3', foreground='#45ff9a')
        log.tag_config('time', foreground='#6f8495')

        lines = []

        def append_line(timestamp: str, source: str, text: str) -> None:
            lines.append((timestamp, source, text))
            if len(lines) > self.max_lines:
                del lines[:len(lines) - self.max_lines]

            log.configure(state='normal')
            log.delete('1.0', 'end')
            for line_time, line_source, line_text in lines:
                log.insert('end', f'[{line_time}] ', 'time')
                log.insert('end', f'{line_source:<22} ', line_source)
                log.insert('end', f'{line_text}\n')
            log.see('end')
            log.configure(state='disabled')

        def poll_queue() -> None:
            while True:
                try:
                    timestamp, source, text = self.event_queue.get_nowait()
                except queue.Empty:
                    break
                append_line(timestamp, source, text)
            root.after(120, poll_queue)

        append_line(datetime.now().strftime('%H:%M:%S'), 'QWEN', 'dialog_panel_ready')
        root.after(120, poll_queue)
        root.mainloop()


def main(args: Optional[list] = None) -> None:
    rclpy.init(args=args)
    node = QwenDialogPanel()
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
