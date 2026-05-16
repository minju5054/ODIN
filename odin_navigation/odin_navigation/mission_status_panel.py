import os
import queue
import threading
from typing import Optional

import rclpy
from odin_interfaces.msg import HostageEvent
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from std_msgs.msg import String


class MissionStatusPanel(Node):
    """Show a desktop mission timeline for the ODIN demo flow."""

    def __init__(self) -> None:
        super().__init__('mission_status_panel')

        self.declare_parameter('hostage_event_topic', '/hostage_events')
        self.declare_parameter('coordinator_status_topic', '/coordinator/status')
        self.declare_parameter('dispatch_status_topic', '/robot_3/dispatch_status')
        self.declare_parameter('mission_status_topic', '/mission_status')
        self.declare_parameter('window_title', 'ODIN Mission Timeline')

        self.window_title = str(self.get_parameter('window_title').value)
        self.detecting_robot = ''
        self.stage_index = 0
        self.close_requested = False
        self.event_queue: queue.Queue = queue.Queue()
        self.tk_available = bool(os.environ.get('DISPLAY'))

        if self.tk_available:
            thread = threading.Thread(target=self._run_tk, daemon=True)
            thread.start()
        else:
            self.get_logger().warning(
                'DISPLAY is not set; mission status panel is disabled for this session.'
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
            str(self.get_parameter('dispatch_status_topic').value),
            self._dispatch_status_callback,
            10,
        )
        self.create_subscription(
            String,
            str(self.get_parameter('mission_status_topic').value),
            self._mission_status_callback,
            10,
        )

        self.get_logger().info('Mission status panel ready.')

    def _hostage_event_callback(self, msg: HostageEvent) -> None:
        self.detecting_robot = self._format_robot_name(msg.detecting_robot)
        self._set_stage(1)

    def _coordinator_status_callback(self, msg: String) -> None:
        if msg.data == 'coordinator_ready':
            return
        self._set_stage(2)

    def _dispatch_status_callback(self, msg: String) -> None:
        if (
            'spawn_trigger_received' in msg.data
            or 'spawn_requested' in msg.data
            or 'spawn_succeeded' in msg.data
            or 'nav2_goal_queued' in msg.data
            or 'nav2_goal_sent' in msg.data
            or 'nav2_goal_accepted' in msg.data
            or 'nav2_feedback' in msg.data
        ):
            self._set_stage(3)

        if 'nav2_result status=4' in msg.data or 'goal_reached' in msg.data:
            self._set_stage(4)

    def _mission_status_callback(self, msg: String) -> None:
        if msg.data == 'mission_success':
            self._set_stage(4)

    def _set_stage(self, stage_index: int) -> None:
        if stage_index < self.stage_index:
            return
        self.stage_index = stage_index
        if self.tk_available:
            self.event_queue.put((self.stage_index, self.detecting_robot))
        if stage_index >= 4:
            self.close_requested = True

    @staticmethod
    def _format_robot_name(robot_name: str) -> str:
        if not robot_name:
            return 'ROBOT'
        return robot_name.replace('robot_', 'ROBOT-').replace('robot', 'ROBOT').upper()

    def _run_tk(self) -> None:
        try:
            import tkinter as tk
        except ImportError:
            self.get_logger().warning(
                'python3-tk is not installed; cannot show mission status panel.'
            )
            return

        root = tk.Tk()
        root.title(self.window_title)
        root.configure(bg='#071018')
        root.geometry('560x340')
        root.attributes('-topmost', True)

        container = tk.Frame(root, bg='#071018', padx=22, pady=18)
        container.pack(expand=True, fill='both')

        title = tk.Label(
            container,
            text='ODIN MISSION TIMELINE',
            fg='#d7ecff',
            bg='#071018',
            font=('DejaVu Sans', 20, 'bold'),
        )
        title.pack(anchor='w', pady=(0, 12))

        cards = []
        for text in self._step_texts(''):
            frame = tk.Frame(container, bg='#14202b', padx=12, pady=8)
            frame.pack(fill='x', pady=4)
            marker = tk.Label(
                frame,
                text=' ',
                width=7,
                anchor='w',
                fg='#6b7f8f',
                bg='#14202b',
                font=('DejaVu Sans', 11, 'bold'),
            )
            marker.pack(side='left')
            label = tk.Label(
                frame,
                text=text,
                anchor='w',
                fg='#8fa3b2',
                bg='#14202b',
                font=('DejaVu Sans', 14, 'bold'),
            )
            label.pack(side='left', fill='x', expand=True)
            cards.append((frame, marker, label))

        def apply_state(stage_index: int, detecting_robot: str) -> None:
            for index, text in enumerate(self._step_texts(detecting_robot)):
                frame, marker, label = cards[index]
                label.configure(text=text)
                if index < stage_index:
                    frame.configure(bg='#10291d')
                    marker.configure(text='DONE', fg='#45ff9a', bg='#10291d')
                    label.configure(fg='#c9ffdf', bg='#10291d')
                elif index == stage_index:
                    frame.configure(bg='#203650')
                    marker.configure(text='ACTIVE', fg='#72b7ff', bg='#203650')
                    label.configure(fg='#ffffff', bg='#203650')
                else:
                    frame.configure(bg='#14202b')
                    marker.configure(text='WAIT', fg='#6b7f8f', bg='#14202b')
                    label.configure(fg='#8fa3b2', bg='#14202b')

        def poll_queue() -> None:
            should_close = False
            while True:
                try:
                    stage_index, detecting_robot = self.event_queue.get_nowait()
                except queue.Empty:
                    break
                apply_state(stage_index, detecting_robot)
                if stage_index >= 4:
                    should_close = True
            if should_close or self.close_requested:
                root.after(700, root.destroy)
                return
            root.after(100, poll_queue)

        apply_state(0, '')
        root.after(100, poll_queue)
        root.mainloop()

    @staticmethod
    def _step_texts(detecting_robot: str):
        detector = detecting_robot if detecting_robot else 'ROBOT'
        return [
            'SCOUTING',
            f'{detector} DETECTED HOSTAGE',
            'COORDINATOR VALIDATING',
            'ROBOT-3 DISPATCHED',
            'MISSION SUCCESS',
        ]


def main(args: Optional[list] = None) -> None:
    rclpy.init(args=args)
    node = MissionStatusPanel()
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
