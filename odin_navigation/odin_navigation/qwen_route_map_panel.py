import os
import queue
import threading
from typing import List, Optional, Tuple

import rclpy
from nav_msgs.msg import OccupancyGrid, Odometry, Path
from odin_interfaces.msg import HostageEvent
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from visualization_msgs.msg import MarkerArray


Point2D = Tuple[float, float]


class QwenRouteMapPanel(Node):
    """Show merged map, candidate routes, and the virtual Qwen selected route."""

    def __init__(self) -> None:
        super().__init__('qwen_route_map_panel')

        self.declare_parameter('merged_map_topic', '/merged_map')
        self.declare_parameter('hostage_event_topic', '/hostage_events')
        self.declare_parameter('coordinator_path_topic', '/coordinator/candidate_path')
        self.declare_parameter('candidate_routes_topic', '/coordinator/candidate_routes')
        self.declare_parameter('selected_path_topic', '/ai/selected_path')
        self.declare_parameter('robot_3_odom_topic', '/robot_3/odom')
        self.declare_parameter('red_zone_min_x', 4.0)
        self.declare_parameter('red_zone_max_x', 10.0)
        self.declare_parameter('red_zone_min_y', 4.0)
        self.declare_parameter('red_zone_max_y', 10.0)
        self.declare_parameter('window_title', 'ODIN Qwen Route Decision')

        self.window_title = str(self.get_parameter('window_title').value)
        self.red_zone = (
            float(self.get_parameter('red_zone_min_x').value),
            float(self.get_parameter('red_zone_max_x').value),
            float(self.get_parameter('red_zone_min_y').value),
            float(self.get_parameter('red_zone_max_y').value),
        )
        self.event_queue: queue.Queue = queue.Queue()
        self.tk_available = bool(os.environ.get('DISPLAY'))

        if self.tk_available:
            thread = threading.Thread(target=self._run_tk, daemon=True)
            thread.start()
        else:
            self.get_logger().warning(
                'DISPLAY is not set; Qwen route map panel is disabled for this session.'
            )

        self.create_subscription(
            HostageEvent,
            str(self.get_parameter('hostage_event_topic').value),
            self._hostage_event_callback,
            10,
        )
        self.create_subscription(
            Odometry,
            str(self.get_parameter('robot_3_odom_topic').value),
            self._robot_3_odom_callback,
            10,
        )
        self.create_subscription(
            OccupancyGrid,
            str(self.get_parameter('merged_map_topic').value),
            self._map_callback,
            1,
        )
        self.create_subscription(
            Path,
            str(self.get_parameter('coordinator_path_topic').value),
            self._coordinator_path_callback,
            10,
        )
        self.create_subscription(
            MarkerArray,
            str(self.get_parameter('candidate_routes_topic').value),
            self._candidate_routes_callback,
            10,
        )
        self.create_subscription(
            Path,
            str(self.get_parameter('selected_path_topic').value),
            self._selected_path_callback,
            10,
        )
        self.get_logger().info('Qwen route map panel ready.')

    def _map_callback(self, msg: OccupancyGrid) -> None:
        self.event_queue.put(('map', msg))

    def _hostage_event_callback(self, msg: HostageEvent) -> None:
        self.event_queue.put(('hostage_point', (msg.pose.position.x, msg.pose.position.y)))

    def _robot_3_odom_callback(self, msg: Odometry) -> None:
        pose = msg.pose.pose
        self.event_queue.put(('robot_3_pose', (pose.position.x, pose.position.y)))

    def _coordinator_path_callback(self, msg: Path) -> None:
        self.event_queue.put(('coordinator_path', self._path_points(msg)))

    def _candidate_routes_callback(self, msg: MarkerArray) -> None:
        routes: List[List[Point2D]] = []
        for marker in msg.markers:
            if marker.action == marker.DELETEALL:
                routes.clear()
                continue
            if marker.type != marker.LINE_STRIP:
                continue
            routes.append([(point.x, point.y) for point in marker.points])
        self.event_queue.put(('candidate_routes', routes))

    def _selected_path_callback(self, msg: Path) -> None:
        self.event_queue.put(('selected_path', self._path_points(msg)))

    @staticmethod
    def _path_points(path: Path) -> List[Point2D]:
        return [(pose.pose.position.x, pose.pose.position.y) for pose in path.poses]

    def _run_tk(self) -> None:
        try:
            import tkinter as tk
        except ImportError:
            self.get_logger().warning(
                'python3-tk is not installed; cannot show Qwen route map panel.'
            )
            return

        root = tk.Tk()
        root.title(self.window_title)
        root.configure(bg='#071018')
        root.geometry('700x760')
        root.attributes('-topmost', True)

        canvas_size = 620
        container = tk.Frame(root, bg='#071018', padx=20, pady=16)
        container.pack(expand=True, fill='both')

        title = tk.Label(
            container,
            text='QWEN ROUTE DECISION MAP',
            fg='#d7ecff',
            bg='#071018',
            font=('DejaVu Sans', 19, 'bold'),
        )
        title.pack(anchor='w', pady=(0, 6))

        legend = tk.Label(
            container,
            text='gray: candidate routes   cyan: selected route   red: robot-3 trail / hostage',
            fg='#9fb2c1',
            bg='#071018',
            font=('DejaVu Sans', 10, 'bold'),
        )
        legend.pack(anchor='w', pady=(0, 8))

        canvas = tk.Canvas(
            container,
            width=canvas_size,
            height=canvas_size,
            bg='#2f3336',
            highlightthickness=0,
        )
        canvas.pack()

        state = {
            'map': None,
            'photo': None,
            'coordinator_path': [],
            'candidate_routes': [],
            'selected_path': [],
            'hostage_point': None,
            'robot_3_trail': [],
        }

        def world_to_canvas(grid: OccupancyGrid, x: float, y: float) -> Tuple[float, float]:
            origin = grid.info.origin.position
            scale = 1.0 / max(grid.info.resolution, 0.001)
            margin_x = (canvas_size - grid.info.width) / 2.0
            margin_y = (canvas_size - grid.info.height) / 2.0
            canvas_x = margin_x + (x - origin.x) * scale
            canvas_y = canvas_size - margin_y - (y - origin.y) * scale
            return canvas_x, canvas_y

        def make_photo(grid: OccupancyGrid):
            photo = tk.PhotoImage(width=grid.info.width, height=grid.info.height)
            rows = []
            for y in reversed(range(grid.info.height)):
                row = []
                offset = y * grid.info.width
                for value in grid.data[offset:offset + grid.info.width]:
                    if value < 0:
                        row.append('#697d79')
                    elif value >= 65:
                        row.append('#111111')
                    else:
                        row.append('#e8ecec')
                rows.append('{' + ' '.join(row) + '}')
            photo.put(' '.join(rows))
            return photo

        def draw_polyline(grid: OccupancyGrid, points: List[Point2D], color: str, width: int) -> None:
            if len(points) < 2:
                return
            coords = []
            for x, y in points:
                coords.extend(world_to_canvas(grid, x, y))
            canvas.create_line(*coords, fill=color, width=width, smooth=True)
            radius = max(3, width + 1)
            for x, y in points[::max(1, len(points) // 12)]:
                cx, cy = world_to_canvas(grid, x, y)
                canvas.create_oval(
                    cx - radius,
                    cy - radius,
                    cx + radius,
                    cy + radius,
                    fill=color,
                    outline='',
                )

        def draw_red_zone(grid: OccupancyGrid) -> None:
            min_x, max_x, min_y, max_y = self.red_zone
            x1, y1 = world_to_canvas(grid, min_x, min_y)
            x2, y2 = world_to_canvas(grid, max_x, max_y)
            canvas.create_rectangle(
                min(x1, x2),
                min(y1, y2),
                max(x1, x2),
                max(y1, y2),
                fill='#ff3333',
                outline='#ff5a5a',
                width=2,
                stipple='gray50',
            )
            canvas.create_text(
                min(x1, x2) + 10,
                min(y1, y2) + 16,
                text='ENEMY AREA',
                fill='#ffdddd',
                anchor='w',
                font=('DejaVu Sans', 12, 'bold'),
            )

        def draw_hostage_point(grid: OccupancyGrid, point: Optional[Point2D]) -> None:
            if point is None:
                return
            x, y = point
            cx, cy = world_to_canvas(grid, x, y)
            radius = 8
            canvas.create_oval(
                cx - radius,
                cy - radius,
                cx + radius,
                cy + radius,
                fill='#ff2626',
                outline='#ffffff',
                width=2,
            )
            canvas.create_text(
                cx + 12,
                cy - 12,
                text=f'HOSTAGE ({x:.2f}, {y:.2f})',
                fill='#ff3b3b',
                anchor='w',
                font=('DejaVu Sans', 12, 'bold'),
            )

        def draw_robot_3_trail(grid: OccupancyGrid, points: List[Point2D]) -> None:
            if len(points) < 2:
                return
            draw_polyline(grid, points, '#ff2d2d', 4)
            x, y = points[-1]
            cx, cy = world_to_canvas(grid, x, y)
            radius = 6
            canvas.create_oval(
                cx - radius,
                cy - radius,
                cx + radius,
                cy + radius,
                fill='#ff2d2d',
                outline='#ffffff',
                width=2,
            )

        def redraw() -> None:
            canvas.delete('all')
            grid: Optional[OccupancyGrid] = state['map']
            if grid is None:
                canvas.create_text(
                    canvas_size / 2,
                    canvas_size / 2,
                    text='WAITING FOR /merged_map',
                    fill='#d7ecff',
                    font=('DejaVu Sans', 22, 'bold'),
                )
                return

            photo = make_photo(grid)
            state['photo'] = photo
            canvas.create_image(
                canvas_size / 2,
                canvas_size / 2,
                image=photo,
                anchor='center',
            )
            canvas.create_rectangle(
                (canvas_size - grid.info.width) / 2,
                (canvas_size - grid.info.height) / 2,
                (canvas_size + grid.info.width) / 2,
                (canvas_size + grid.info.height) / 2,
                outline='#7d8b91',
                width=2,
            )

            draw_red_zone(grid)
            for route in state['candidate_routes']:
                draw_polyline(grid, route, '#a7aaad', 2)
            if state['coordinator_path']:
                draw_polyline(grid, state['coordinator_path'], '#7f8589', 2)
            if state['selected_path']:
                draw_polyline(grid, state['selected_path'], '#00e5ff', 5)
            draw_robot_3_trail(grid, state['robot_3_trail'])
            draw_hostage_point(grid, state['hostage_point'])

        def poll_queue() -> None:
            changed = False
            while True:
                try:
                    event_name, payload = self.event_queue.get_nowait()
                except queue.Empty:
                    break
                if event_name == 'robot_3_pose':
                    trail = state['robot_3_trail']
                    if not trail or abs(trail[-1][0] - payload[0]) + abs(trail[-1][1] - payload[1]) > 0.05:
                        trail.append(payload)
                        if len(trail) > 700:
                            del trail[:len(trail) - 700]
                else:
                    state[event_name] = payload
                changed = True
            if changed:
                redraw()
            root.after(120, poll_queue)

        redraw()
        root.after(120, poll_queue)
        root.mainloop()


def main(args: Optional[list] = None) -> None:
    rclpy.init(args=args)
    node = QwenRouteMapPanel()
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
