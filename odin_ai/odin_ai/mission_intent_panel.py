import os
import queue
import threading
from typing import Optional

import rclpy
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from std_msgs.msg import String


class MissionIntentPanel(Node):
    """Small desktop input panel for choosing the mission intent before dispatch."""

    def __init__(self) -> None:
        super().__init__('mission_intent_panel')

        self.declare_parameter('mission_intent_topic', '/mission/intent')
        self.declare_parameter('mission_policy_topic', '/ai/mission_policy')
        self.declare_parameter('window_title', 'ODIN Mission Intent')

        self.window_title = str(self.get_parameter('window_title').value)
        self.intent_queue: queue.Queue = queue.Queue()
        self.policy_queue: queue.Queue = queue.Queue()
        self.tk_available = bool(os.environ.get('DISPLAY'))

        self.intent_pub = self.create_publisher(
            String,
            str(self.get_parameter('mission_intent_topic').value),
            10,
        )
        self.create_subscription(
            String,
            str(self.get_parameter('mission_policy_topic').value),
            self._mission_policy_callback,
            10,
        )
        self.create_timer(0.1, self._publish_queued_intents)

        if self.tk_available:
            thread = threading.Thread(target=self._run_tk, daemon=True)
            thread.start()
        else:
            self.get_logger().warning(
                'DISPLAY is not set; mission intent panel is disabled for this session.'
            )

        self.get_logger().info('Mission intent panel ready.')

    def _mission_policy_callback(self, msg: String) -> None:
        if self.tk_available:
            self.policy_queue.put(msg.data)

    def _publish_queued_intents(self) -> None:
        while True:
            try:
                intent = self.intent_queue.get_nowait()
            except queue.Empty:
                break
            msg = String()
            msg.data = intent
            self.intent_pub.publish(msg)
            self.get_logger().info(f'mission_intent_sent text={intent}')

    def _run_tk(self) -> None:
        try:
            import tkinter as tk
            from tkinter import scrolledtext
        except ImportError:
            self.get_logger().warning(
                'python3-tk is not installed; cannot show mission intent panel.'
            )
            return

        root = tk.Tk()
        root.title(self.window_title)
        root.configure(bg='#081018')
        root.geometry('560x420')
        root.attributes('-topmost', True)

        container = tk.Frame(root, bg='#081018', padx=22, pady=18)
        container.pack(expand=True, fill='both')

        title = tk.Label(
            container,
            text='MISSION INTENT',
            fg='#d7ecff',
            bg='#081018',
            font=('DejaVu Sans', 22, 'bold'),
        )
        title.pack(anchor='w')

        subtitle = tk.Label(
            container,
            text='Describe the situation. Qwen will choose FAST, SAFE, or STEALTH policy.',
            fg='#8fa3b2',
            bg='#081018',
            font=('DejaVu Sans', 11, 'bold'),
        )
        subtitle.pack(anchor='w', pady=(2, 12))

        text_box = scrolledtext.ScrolledText(
            container,
            height=5,
            bg='#0d1b28',
            fg='#ecf7ff',
            insertbackground='#ecf7ff',
            font=('DejaVu Sans Mono', 11),
            wrap='word',
        )
        text_box.pack(fill='x')
        text_box.insert('1.0', '안전하게 구출해야 해. 이미 정찰된 경로를 우선해.')

        status_var = tk.StringVar(value='Policy: waiting for mission intent')
        status = tk.Label(
            container,
            textvariable=status_var,
            fg='#33ff88',
            bg='#081018',
            font=('DejaVu Sans', 13, 'bold'),
        )
        status.pack(anchor='w', pady=(12, 8))

        def set_intent_text(text: str) -> None:
            text_box.delete('1.0', 'end')
            text_box.insert('1.0', text)
            status_var.set('Policy: ready to send intent')

        def submit_intent(text: str) -> None:
            intent = text.strip()
            if not intent:
                return
            self.intent_queue.put(intent)
            status_var.set('Policy: Qwen is evaluating...')

        submit_button = tk.Button(
            container,
            text='Send Intent To Qwen',
            command=lambda: submit_intent(text_box.get('1.0', 'end')),
            bg='#1c7ed6',
            fg='white',
            activebackground='#339af0',
            activeforeground='white',
            font=('DejaVu Sans', 12, 'bold'),
            relief='flat',
            padx=14,
            pady=8,
        )
        submit_button.pack(anchor='w', pady=(0, 12))

        presets = tk.Frame(container, bg='#081018')
        presets.pack(fill='x')

        preset_items = [
            ('FAST', '빨리 구출해야 해. 인질 생존 시간이 가장 중요해.'),
            ('SAFE', '안전하게 구출해야 해. 알려진 정찰 경로를 우선해.'),
            ('STEALTH', '절대 적에게 들키면 안 돼. 은밀하게 접근해야 해.'),
        ]
        for label, text in preset_items:
            button = tk.Button(
                presets,
                text=label,
                command=lambda value=text: set_intent_text(value),
                bg='#172432',
                fg='#d7ecff',
                activebackground='#22364a',
                activeforeground='white',
                font=('DejaVu Sans', 11, 'bold'),
                relief='flat',
                padx=12,
                pady=7,
            )
            button.pack(side='left', padx=(0, 8))

        def poll_policy() -> None:
            while True:
                try:
                    policy = self.policy_queue.get_nowait()
                except queue.Empty:
                    break
                status_var.set(f'Policy: {policy}')
            root.after(100, poll_policy)

        root.after(100, poll_policy)
        root.mainloop()


def main(args: Optional[list] = None) -> None:
    rclpy.init(args=args)
    node = MissionIntentPanel()
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
