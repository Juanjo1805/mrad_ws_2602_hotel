import rclpy
from rclpy.node import Node
from geometry_msgs.msg import PoseStamped
from nav_msgs.msg import OccupancyGrid, Path
import tf2_ros
import tf_transformations
import numpy as np
import heapq
from rclpy.qos import QoSProfile, QoSDurabilityPolicy, QoSReliabilityPolicy, QoSHistoryPolicy

class DijkstraNode(Node):
    def __init__(self):
        super().__init__('dijkstra_node')
        
        # --- add these params at __init__ ---
        self.occ_thresh = self.declare_parameter('occ_thresh', 100).value
        self.robot_radius_m = self.declare_parameter('robot_radius_m', 0.5).value
        self.safety_margin_m = self.declare_parameter('safety_margin_m', 0.05).value
        map_qos = QoSProfile(
            reliability=QoSReliabilityPolicy.RELIABLE,
            durability=QoSDurabilityPolicy.TRANSIENT_LOCAL,
            history=QoSHistoryPolicy.KEEP_LAST,
            depth=1
        )

        # Subscribers
        self.create_subscription(
            OccupancyGrid,
            '/map',
            self.map_callback,
            map_qos
        )
        self.create_subscription(PoseStamped, '/goal_pose', self.goal_callback, 10)

        # Publishers
        self.path_pub = self.create_publisher(Path, '/planned_path', map_qos)

        # tf2 listener
        self.tf_buffer = tf2_ros.Buffer()
        self.tf_listener = tf2_ros.TransformListener(self.tf_buffer, self)

        # Storage
        self.map = None
        self.goal = None
        print('Dijkstra Node Initialized')
        

    def map_callback(self, msg):
        self.map = msg
        
        # 2) compute how many cells to inflate
        res = self.map.info.resolution
        inflation_cells = max(0, int(np.floor((self.robot_radius_m + self.safety_margin_m) / res)))

        # 3) make mask of cells to inflate (True = will become occupied)
        inflated_mask = self.make_inflation_mask(self.map, inflation_cells, self.occ_thresh)

        # 4) write 100s into the map data (hard inflation)
        w, h = self.map.info.width, self.map.info.height
        data = np.array(self.map.data, dtype=np.int16).reshape((h, w))
        data[inflated_mask] = 100  # or 254, anything >= occ_thresh
        self.map.data = data.flatten().tolist()
        
        print('Map received')
		
    def make_inflation_mask(self, occ_grid, inflation_cells: int, occ_thresh: int):
        """Return boolean mask (h,w): True where cells are within `inflation_cells` of any obstacle."""
        w, h = occ_grid.info.width, occ_grid.info.height
        data = np.array(occ_grid.data, dtype=np.int16).reshape((h, w))
        occ = (data >= occ_thresh).astype(np.uint8)

        # Multi-source BFS (brushfire) distance in cells
        INF = 1_000_000
        dist = np.full((h, w), INF, dtype=np.int32)

        from collections import deque
        q = deque()
        ys, xs = np.where(occ == 1)
        for y, x in zip(ys, xs):
            dist[y, x] = 0
            q.append((x, y))

        # 4-connected neighbors for distance
        for_yx = [(-1,0),(1,0),(0,-1),(0,1)]
        while q:
            x, y = q.popleft()
            d = dist[y, x]
            for dx, dy in for_yx:
                nx, ny = x + dx, y + dy
                if 0 <= nx < w and 0 <= ny < h and dist[ny, nx] > d + 1:
                    dist[ny, nx] = d + 1
                    q.append((nx, ny))

        # Inflate: cells with distance <= inflation_cells become occupied
        return dist <= inflation_cells
        
    def goal_callback(self, msg):
        self.goal = msg
        print('Goal received')
        self.plan_path()
		
    def plan_path(self):
        if not self.map or not self.goal:
            return

        try:
            tf = self.tf_buffer.lookup_transform('map', 'base_link', rclpy.time.Time())
        except Exception as e:
            self.get_logger().warn(f"TF error: {e}")
            return

        # Convert start and goal to grid coordinates
        start = self.world_to_grid(tf.transform.translation)
        goal = self.world_to_grid(self.goal.pose.position)

        path = self.dijkstra(start, goal)

        ros_path = Path()
        ros_path.header.frame_id = 'map'
        for (x, y) in path:
            pose = PoseStamped()
            pose.header.frame_id = 'map'
            pose.pose.position.x = x * self.map.info.resolution + self.map.info.origin.position.x
            pose.pose.position.y = y * self.map.info.resolution + self.map.info.origin.position.y
            pose.pose.orientation.w = 1.0
            ros_path.poses.append(pose)

        self.path_pub.publish(ros_path)
    
    def world_to_grid(self, pos):
        mx = int((pos.x - self.map.info.origin.position.x) / self.map.info.resolution)
        my = int((pos.y - self.map.info.origin.position.y) / self.map.info.resolution)
        return (mx, my)
    
    def dijkstra(self, start, goal):
        width = self.map.info.width
        height = self.map.info.height
        data = np.array(self.map.data).reshape((height, width))

        def in_bounds(x, y):
            return 0 <= x < width and 0 <= y < height and data[y][x] < 50

        visited = set()
        queue = [(0, start)]
        came_from = {start: None}

        while queue:
            cost, current = heapq.heappop(queue)

            if current in visited:
                continue
            visited.add(current)

            if current == goal:
                break

            x, y = current
            for dx, dy in [(-1,0),(1,0),(0,-1),(0,1)]:
                nx, ny = x + dx, y + dy
                neighbor = (nx, ny)
                if in_bounds(nx, ny) and neighbor not in visited:
                    heapq.heappush(queue, (cost + 1, neighbor))
                    if neighbor not in came_from:
                        came_from[neighbor] = current

        # Reconstruct path
        path = []
        node = goal
        while node:
            path.append(node)
            node = came_from.get(node)
        path.reverse()
        return path

	

def main(args=None):
    rclpy.init(args=args)
    node = DijkstraNode()
    rclpy.spin(node)
    rclpy.shutdown()
	

if __name__ == '__main__':    main()