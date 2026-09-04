#!/usr/bin/env python3
"""
ARA* (Anytime Repairing A*) grid planner (ROS 2).
Basado en la estructura de Dijkstra proporcionada en clase.
"""
import math
import heapq
import time
from collections import deque
from typing import List, Tuple, Optional, Dict, Set

import numpy as np

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, DurabilityPolicy

from nav_msgs.msg import OccupancyGrid, Path
from geometry_msgs.msg import PoseStamped, Quaternion
from tf2_geometry_msgs import do_transform_pose
import tf2_ros
from dataclasses import dataclass, field
from typing import Tuple, Optional
from visualization_msgs.msg import Marker, MarkerArray
from geometry_msgs.msg import Point
from std_msgs.msg import ColorRGBA
from nav_msgs.srv import GetPlan

@dataclass
class ARANode:
    x: int
    y: int
    g: float = float('inf')
    v: float = float('inf')
    parent: Optional[Tuple[int, int]] = None
    
    # Mantenemos el __lt__ para heapq
    def __lt__(self, other):
        return False

def yaw_to_quaternion(yaw: float) -> Quaternion:
    q = Quaternion()
    q.w = math.cos(yaw / 2.0)
    q.z = math.sin(yaw / 2.0)
    return q

class ARAPlannerNode(Node):
    def __init__(self):
        super().__init__('ara_path_planner')

        # --- Parámetros de ROS 2 (Reciclados de Dijkstra) ---
        self.declare_parameter('topics.map_topic', '/map')
        self.declare_parameter('goal_source', 'mission')
        self.declare_parameter('topics.goal_topic', '/goal_pose')
        self.declare_parameter('topics.path_topic', '/planned_path')
        self.declare_parameter('plan_service', '/get_plan')

        self.declare_parameter('frames.base_frame', 'base_link')
        self.declare_parameter('frames.global_frame', 'map')
        self.declare_parameter('topics.debug_paths', '/ara_debug_paths')  # Nuevo tópico para rutas de depuración
        
        # Opciones de grilla
        self.declare_parameter('geometry.occupied_threshold', 60)
        self.declare_parameter('geometry.use_8_connected', True)
        self.declare_parameter('geometry.inflate_radius', 0.60)
        self.declare_parameter('geometry.treat_unknown_as_obstacle', True)

        # --- Parámetros NUEVOS para ARA* ---
        self.declare_parameter('ara_core.epsilon_start', 2.8)       # Inflación inicial (Modo rápido)
        self.declare_parameter('ara_core.epsilon_decrease', 0.7)    # Cuánto baja en cada iteración
        self.declare_parameter('ara_core.time_limit_sec', 0.6)      # Presupuesto de tiempo total
        self.declare_parameter('ara_core.heuristic_type', 'euclidean')

        self.declare_parameter('debug.publish_all_paths', False)
        self._debug_mode = self.get_parameter('debug.publish_all_paths').get_parameter_value().bool_value      

        self._heuristic_type = self.get_parameter('ara_core.heuristic_type').get_parameter_value().string_value.lower()
        self._use_8_conn = self.get_parameter('geometry.use_8_connected').get_parameter_value().bool_value

        # PRE-CALCULAR LOS MOVIMIENTOS UNA SOLA VEZ
        straight_moves = [(0, 1, 1.0), (0, -1, 1.0), (1, 0, 1.0), (-1, 0, 1.0)]
        diagonal_moves = [(1, 1, 1.4142), (-1, 1, 1.4142), (1, -1, 1.4142), (-1, -1, 1.4142)]
        self._allowed_moves = straight_moves + diagonal_moves if self._use_8_conn else straight_moves

        # --- Subscripciones y Publicadores ---
        map_topic = self.get_parameter('topics.map_topic').get_parameter_value().string_value
        goal_topic = self.get_parameter('topics.goal_topic').get_parameter_value().string_value
        path_topic = self.get_parameter('topics.path_topic').get_parameter_value().string_value
        debug_topic = self.get_parameter('topics.debug_paths').get_parameter_value().string_value
        plan_srv_name = self.get_parameter('plan_service').get_parameter_value().string_value
        self.goal_source = self.get_parameter('goal_source').value

        self.goal_rviz_sub = self.create_subscription(
            PoseStamped,
            '/goal_pose',
            self.goal_rviz_cb,
            10
        )

        self.goal_mission_sub = self.create_subscription(
            PoseStamped,
            '/mission_goal',
            self.goal_mission_cb,
            10
        )
        self.path_pub = self.create_publisher(Path, path_topic, 10)
        self.debug_paths_pub = self.create_publisher(MarkerArray, debug_topic, 10)
        self.plan_srv = self.create_service(GetPlan, plan_srv_name, self.get_plan_cb)

        qos_map = QoSProfile(
            depth=1,
            reliability=ReliabilityPolicy.RELIABLE,
            durability=DurabilityPolicy.TRANSIENT_LOCAL,
        )
        self.map_sub = self.create_subscription(OccupancyGrid, map_topic, self.map_cb, qos_map)

        # --- TF2 (Reciclado) ---
        self.tf_buffer = tf2_ros.Buffer()
        self.tf_listener = tf2_ros.TransformListener(self.tf_buffer, self)

        # Variables internas
        self._map: Optional[OccupancyGrid] = None
        self._obstacles: Optional[np.ndarray] = None
        self._dist_cells: Optional[np.ndarray] = None
        if self._debug_mode:
            self.get_logger().info("ARA* Planner Node Iniciado con los siguientes parámetros:" \
            f"- Epsilon Start: {self.get_parameter('ara_core.epsilon_start').get_parameter_value().double_value}" \
            f"- Epsilon Decrease: {self.get_parameter('ara_core.epsilon_decrease').get_parameter_value().double_value}" \
            f"- Time Limit (sec): {self.get_parameter('ara_core.time_limit_sec').get_parameter_value().double_value}" \
            f"- Heuristic Type: {self.get_parameter('ara_core.heuristic_type').get_parameter_value().string_value}" \
            f"- Use 8-Connected: {self.get_parameter('geometry.use_8_connected').get_parameter_value().bool_value}" \
            f"- Inflate Radius: {self.get_parameter('geometry.inflate_radius').get_parameter_value().double_value}")

        self.get_logger().info("ARA* Planner Node Iniciado y esperando el mapa...")
        self.get_logger().info(f"Servicio de ARA* activo en: {plan_srv_name}")

    # =================================================================
    # CALLBACKS DE ROS 2
    # =================================================================
    def map_cb(self, msg: OccupancyGrid):
        """Recibe el mapa, lo guarda y pre-calcula los obstáculos (Brushfire)."""
        self._map = msg
        W = msg.info.width
        H = msg.info.height
        res = msg.info.resolution

        grid = np.array(msg.data, dtype=np.int16).reshape((H, W))  # row-major: y first
        self._grid = grid

        occ_th = self.get_parameter('geometry.occupied_threshold').get_parameter_value().integer_value
        unknown_as_obs = self.get_parameter('geometry.treat_unknown_as_obstacle').get_parameter_value().bool_value

        obstacles = (grid >= occ_th)
        if unknown_as_obs:
            obstacles = np.logical_or(obstacles, grid == -1)

        # Precompute distance-to-obstacle field (in cells) from the raw obstacles.
        # This is useful for both inflation and soft traversal costs.
        dist_cells = self.compute_distance_to_obstacles(obstacles)
        self._dist_cells = dist_cells

        # Inflate obstacles if requested (uses distance field: dist <= R).
        inflate_radius = float(self.get_parameter('geometry.inflate_radius').get_parameter_value().double_value)
        if inflate_radius > 1e-6:
            inflation_cells = int(math.ceil(inflate_radius / res))
            obstacles = np.logical_or(obstacles, dist_cells <= inflation_cells)

        self._obstacles = obstacles
        self.get_logger().info(f'Map received: {W}x{H}, res={res:.3f} m/px')

    def compute_distance_to_obstacles(self, obstacles: np.ndarray) -> np.ndarray:
        """Brushfire / multi-source BFS distance transform (4-connected).

        Returns dist[y,x] in *cells* to the nearest obstacle cell.
        Obstacle cells have distance 0.
        """
        H, W = obstacles.shape
        INF = np.iinfo(np.int32).max
        dist = np.full((H, W), INF, dtype=np.int32)

        q = deque()
        ys, xs = np.nonzero(obstacles)
        for y, x in zip(ys, xs):
            dist[y, x] = 0
            q.append((x, y))

        # If there are no obstacles, dist stays INF everywhere.
        if not q:
            return dist

        nbr4 = [(-1, 0), (1, 0), (0, -1), (0, 1)]
        while q:
            x, y = q.popleft()
            d = dist[y, x]
            nd = d + 1
            for dx, dy in nbr4:
                nx, ny = x + dx, y + dy
                if 0 <= nx < W and 0 <= ny < H and dist[ny, nx] > nd:
                    dist[ny, nx] = nd
                    q.append((nx, ny))
        return dist

    def goal_rviz_cb(self, msg):

        if self.goal_source != "rviz":
            return

        self.get_logger().info("Goal received from RViz")

        self.process_goal(msg)


    def goal_mission_cb(self, msg):

        if self.goal_source != "mission":
            return

        self.get_logger().info("Goal received from mission manager")

        self.process_goal(msg)
    def process_goal(self, msg: PoseStamped):

        if self._map is None or self._obstacles is None:
            self.get_logger().warn("No hay mapa todavía.")
            return

        try:

            transform = self.tf_buffer.lookup_transform(
                self.get_parameter('frames.global_frame').get_parameter_value().string_value,
                self.get_parameter('frames.base_frame').get_parameter_value().string_value,
                rclpy.time.Time(seconds=0)
            )

            start_pose = PoseStamped()
            start_pose.header.frame_id = self.get_parameter(
                'frames.global_frame'
            ).get_parameter_value().string_value

            start_pose.header.stamp = self.get_clock().now().to_msg()

            start_pose.pose.position.x = transform.transform.translation.x
            start_pose.pose.position.y = transform.transform.translation.y
            start_pose.pose.position.z = 0.0
            start_pose.pose.orientation.w = 1.0

        except Exception as e:

            self.get_logger().error(f"Error obteniendo TF: {e}")
            return

        path_msg = self.plan_ara_star(start_pose, msg)

        if path_msg is not None:

            self.path_pub.publish(path_msg)
            self.get_logger().info("Ruta publicada")

        else:

            self.get_logger().error("ARA* falló")
    
    def get_plan_cb(self, request, response):
        """
        Callback del servicio /get_plan.
        Recibe un Request con (start, goal) y devuelve un Response con (plan).
        """
        self.get_logger().info("¡Solicitud de ruta recibida por Servicio!")

        if self._map is None or self._obstacles is None:
            self.get_logger().warn("No hay mapa todavía. Rechazando solicitud.")
            return response  # Devuelve la respuesta vacía

        # 1. Extraer las poses enviadas por tu compañero
        start_pose = request.start
        goal_pose = request.goal

        # (Opcional pero recomendado) Validación de seguridad: 
        # Si tu compañero envió un start_pose vacío (sin frame_id),
        # usamos el TF del robot como respaldo de emergencia.
        if not start_pose.header.frame_id:
            self.get_logger().warn("El Request no trajo Start Pose. Usando TF del robot...")
            try:
                transform = self.tf_buffer.lookup_transform(
                    self.get_parameter('global_frame').value,
                    self.get_parameter('base_frame').value,
                    rclpy.time.Time()
                )
                start_pose.header.frame_id = self.get_parameter('global_frame').value
                start_pose.pose.position.x = transform.transform.translation.x
                start_pose.pose.position.y = transform.transform.translation.y
            except Exception as e:
                self.get_logger().error(f"Error TF de emergencia: {e}")
                return response

        # 2. ¡Llamar a tu motor matemático intacto!
        path_msg = self.plan_ara_star(start_pose, goal_pose)

        # 3. Empaquetar y devolver la respuesta
        if path_msg is not None:
            response.plan = path_msg
            
            # También lo publicamos en el tópico normal para que 
            # lo puedas ver dibujado en RViz mientras tu compañero hace sus pruebas
            self.path_pub.publish(path_msg)
            
            self.get_logger().info("¡Ruta calculada con éxito y devuelta al cliente!")
        else:
            self.get_logger().error("ARA* falló al encontrar una ruta para el servicio.")

        return response

    # =================================================================
    # 4. FUNCIONES AUXILIARES 
    # =================================================================
    def world_to_map(self, x, y, x0, y0, res, W, H) -> Optional[Tuple[int, int]]:
        """World (meters) -> grid indices (ix, iy)."""
        ix = int(math.floor((x - x0) / res))
        iy = int(math.floor((y - y0) / res))
        if 0 <= ix < W and 0 <= iy < H:
            return ix, iy
        return None

    def map_to_world(self, ix, iy, x0, y0, res) -> Tuple[float, float]:
        """Grid indices -> world (meters), at cell center."""
        x = x0 + (ix + 0.5) * res
        y = y0 + (iy + 0.5) * res
        return x, y

    def get_neighbors(self, ix: int, iy: int, W: int, H: int) -> List[Tuple[int, int, float]]:
        neighbors = []
        
        # Iteramos sobre la lista en memoria, sin crear nuevas listas ni llamar parámetros ROS2
        for dx, dy, cost in self._allowed_moves:
            nx = ix + dx
            ny = iy + dy

            if 0 <= nx < W and 0 <= ny < H:
                if not self._obstacles[ny, nx]:
                    # Chequeo de cruce de esquina (opcional)
                    if abs(dx) == 1 and abs(dy) == 1:
                        if self._obstacles[iy, nx] or self._obstacles[ny, ix]:
                            continue
                    
                    neighbors.append((nx, ny, cost))
        return neighbors

    # =================================================================
    # 5. EL NÚCLEO MATEMÁTICO ARA* 
    # =================================================================
    def calculate_heuristic(self, curr_idx: Tuple[int, int], goal_idx: Tuple[int, int]) -> float:
        """
        Calcula la estimación h(s) desde el nodo actual a la meta.
        Puedes usar distancia Euclidiana o Manhattan.
        """
        # Distancia absoluta en X y en Y
        dx = abs(curr_idx[0] - goal_idx[0])
        dy = abs(curr_idx[1] - goal_idx[1])

        if self._heuristic_type == 'manhattan':
            # Distancia Manhattan: Solo permite movimientos en cruz (L)
            # h(s) = |dx| + |dy|
            return float(dx + dy)
            
        else:
            # Distancia Euclidiana (Por defecto): Línea recta
            # Usamos math.hypot que es más rápido y numéricamente más estable que sqrt(dx**2 + dy**2)
            return math.hypot(dx, dy)

    def f_value(self, g: float, h: float, epsilon: float) -> float:
        """Retorna el costo total estimado: f(s) = g(s) + epsilon * h(s)"""
        return g + (epsilon * h)

    def improve_path(self, goal_idx: Tuple[int, int], epsilon: float, 
                     state_space: Dict[Tuple[int, int], 'ARANode'], 
                     OPEN: list, CLOSED: set, INCONS: set):
        """
        El núcleo de ARA*. Expande nodos hasta que la meta esté garantizada
        para el factor de inflación epsilon actual.
        """
        # 1. Obtener dimensiones del mapa para los vecinos
        W = self._map.info.width
        H = self._map.info.height

        # Asegurarnos de que la meta exista en el state_space para poder evaluar su g(s)
        if goal_idx not in state_space:
            state_space[goal_idx] = ARANode(goal_idx[0], goal_idx[1])

        # =================================================================
        # BUCLE PRINCIPAL (Línea 13 del paper)
        # Condición: Mientras OPEN no esté vacío Y g(meta) > mínimo f(s) en OPEN
        # Nota: f(meta) es igual a g(meta) porque la heurística h(meta) es 0.
        # OPEN[0][0] nos da el f(s) más pequeño actualmente en el min-heap.
        # =================================================================
        while OPEN and state_space[goal_idx].g > OPEN[0][0]:
            
            # Línea 14: Remover el nodo con menor f(s)
            current_f, current_idx = heapq.heappop(OPEN)

            # --- LAZY DELETION ---
            # Si este nodo ya fue expandido en esta iteración, es un "fantasma" 
            # de una actualización anterior. Lo ignoramos.
            if current_idx in CLOSED:
                continue

            current_node = state_space[current_idx]

            # Línea 15: v(s) = g(s) (Hacer el nodo "Consistente")
            current_node.v = current_node.g
            
            # Línea 16: Meterlo a CLOSED
            CLOSED.add(current_idx)

            # Línea 17: Para cada vecino s' del nodo s
            neighbors = self.get_neighbors(current_idx[0], current_idx[1], W, H)
            
            for nx, ny, transition_cost in neighbors:
                neighbor_idx = (nx, ny)
                
                # Inicialización "On-the-fly" (Crear el nodo si no lo habíamos visto nunca)
                if neighbor_idx not in state_space:
                    state_space[neighbor_idx] = ARANode(nx, ny)
                
                neighbor_node = state_space[neighbor_idx]

                # Línea 18: ¿Encontramos un atajo? si g(s') > g(s) + c(s, s')
                new_g = current_node.g + transition_cost
                
                if neighbor_node.g > new_g:
                    
                    # Línea 19: Actualizamos el peso y guardamos el rastro (Parent)
                    neighbor_node.g = new_g
                    neighbor_node.parent = current_idx 

                    # Línea 20: ¿s' NO está en CLOSED?
                    if neighbor_idx not in CLOSED:
                        # Línea 21: Insertar (o actualizar "lazy") s' en OPEN
                        h_val = self.calculate_heuristic(neighbor_idx, goal_idx)
                        new_f = self.f_value(new_g, h_val, epsilon)
                        heapq.heappush(OPEN, (new_f, neighbor_idx))
                    
                    # Línea 22: else (s' SI está en CLOSED, o sea, ya lo habíamos procesado)
                    else:
                        # Línea 23: Insertar s' en INCONS
                        INCONS.add(neighbor_idx)

    def reconstruct_path(self, start_idx, goal_idx, state_space, path_msg_header, x0, y0, res) -> Path:
        """
        Navega hacia atrás usando state_space[nodo].parent para 
        construir el mensaje nav_msgs/Path a publicar en RViz.
        """
        # Crear el mensaje de ROS 2 vacío
        path_msg = Path()
        path_msg.header = path_msg_header

        # ==========================================================
        # 1. Backtracking: Recuperar los índices de la meta al inicio
        # ==========================================================
        current_idx = goal_idx
        path_cells = []

        # Retroceder por los padres hasta llegar a None o al inicio
        while current_idx is not None:
            path_cells.append(current_idx)
            if current_idx == start_idx:
                break
            current_idx = state_space[current_idx].parent

        # Como la lista se construyó desde la meta, la invertimos (Inicio -> Meta)
        path_cells.reverse()

        # ==========================================================
        # 2. Traducción: Matriz -> Mundo Real (Metros)
        # ==========================================================
        last_yaw = 0.0  # Ángulo por defecto
        
        for i, (ix, iy) in enumerate(path_cells):
            # Obtener el centro físico de la celda (usando la función que vimos antes)
            x, y = self.map_to_world(ix, iy, x0, y0, res)
            
            pose = PoseStamped()
            pose.header = path_msg_header
            pose.pose.position.x = x
            pose.pose.position.y = y
            pose.pose.position.z = 0.0

            # ==========================================================
            # 3. Orientación Cinemática (Cálculo del Yaw)
            # ==========================================================
            # Miramos el siguiente punto de la ruta para saber hacia dónde mirar
            if i + 1 < len(path_cells):
                next_ix, next_iy = path_cells[i + 1]
                nx_world, ny_world = self.map_to_world(next_ix, next_iy, x0, y0, res)
                
                # math.atan2 calcula el ángulo del vector entre el punto actual y el siguiente
                last_yaw = math.atan2(ny_world - y, nx_world - x)
            
            # Convertimos el ángulo Yaw de Euler a Cuaternión (Requisito de ROS 2)
            pose.pose.orientation = yaw_to_quaternion(last_yaw)
            
            # Añadir la pose al arreglo final del mensaje Path
            path_msg.poses.append(pose)

        return path_msg

    def plan_ara_star(self, start: PoseStamped, goal: PoseStamped) -> Optional[Path]:
        """
        El procedimiento Main() del paper de Likhachev.
        Controla el presupuesto de tiempo y el ciclo de decremento de epsilon.
        """
        # 1. Extraer metadata del mapa
        info = self._map.info
        res = info.resolution
        x0, y0 = info.origin.position.x, info.origin.position.y
        W, H = info.width, info.height

        # 2. Convertir start y goal a índices
        s_idx = self.world_to_map(start.pose.position.x, start.pose.position.y, x0, y0, res, W, H)
        g_idx = self.world_to_map(goal.pose.position.x, goal.pose.position.y, x0, y0, res, W, H)

        if s_idx is None or g_idx is None:
            self.get_logger().error("Start o Goal fuera del mapa.")
            return None

        # 3. Inicializar parámetros del ARA*
        epsilon = self.get_parameter('ara_core.epsilon_start').value
        eps_dec = self.get_parameter('ara_core.epsilon_decrease').value
        time_limit = self.get_parameter('ara_core.time_limit_sec').value

        # 4. Inicializar estructuras de datos (Las tres listas y el State Space)
        OPEN = []       # Cola de prioridad (heapq)
        CLOSED = set()  # Set para búsqueda rápida
        INCONS = set()  # Nodos a reparar
        
        # Diccionario para guardar todos los objetos ARANode instanciados
        # Llave: Tupla (x,y), Valor: Objeto ARANode
        state_space: Dict[Tuple[int, int], ARANode] = {} 

        # Crear nodo inicial
        start_node = ARANode(s_idx[0], s_idx[1])
        start_node.g = 0.0
        state_space[s_idx] = start_node
        
        # Calcular h(s_start) e insertarlo en OPEN
        h_start = self.calculate_heuristic(s_idx, g_idx)
        f_start = self.f_value(start_node.g, h_start, epsilon)
        heapq.heappush(OPEN, (f_start, s_idx))

        # 5. El Bucle Anytime (Controlado por tiempo y epsilon)
        start_time = time.time()
        best_path_found = False

        debug_markers = MarkerArray()
        iteration_count = 0

        while epsilon >= 1.0:
            self.get_logger().info(f"Buscando ruta con epsilon={epsilon:.2f}...")
            
            # Llamar al motor de expansión
            self.improve_path(g_idx, epsilon, state_space, OPEN, CLOSED, INCONS)

            # Verificar si ImprovePath conectó el inicio con la meta
            if g_idx in state_space and state_space[g_idx].g < float('inf'):
                best_path_found = True
                self.get_logger().info(f"¡Ruta subóptima encontrada para eps={epsilon:.2f}!")
                if self._debug_mode:
                    # Hacemos un mini-backtracking solo de celdas
                    curr = g_idx
                    cells = []
                    while curr is not None:
                        cells.append(curr)
                        if curr == s_idx: break
                        curr = state_space[curr].parent
                    
                    # Crear el marcador visual y guardarlo
                    marker = self.create_path_marker(
                        cells, epsilon, iteration_count, 
                        start.header.frame_id, x0, y0, res
                    )
                    debug_markers.markers.append(marker)
                    iteration_count += 1
            
            # Revisar si se nos acabó el tiempo
            if (time.time() - start_time) > time_limit:
                self.get_logger().warn("Tiempo de cálculo agotado.")
                break
            
            # --- Fase de Reparación ---
            if epsilon == 1.0:
                break # Ya encontramos la óptima, salir del bucle
                
            # Disminuir epsilon
            epsilon -= eps_dec
            if epsilon < 1.0:
                epsilon = 1.0

            # VACIAR INCONS DENTRO DE OPEN Y RECONSTRUIR EL HEAP
            # Necesitamos recalcular f(s) = g(s) + epsilon * h(s) para TODOS los nodos latentes
            
            new_open_list = []
            
            # 1. Recalcular nodos que ya estaban en OPEN
            for _, idx in OPEN:
                # Evitar nodos marcados como lazy deletion
                if idx not in CLOSED:
                    h_val = self.calculate_heuristic(idx, g_idx)
                    new_f = self.f_value(state_space[idx].g, h_val, epsilon)
                    new_open_list.append((new_f, idx))
            
            # 2. Recalcular y agregar nodos de la lista INCONS
            for idx in INCONS:
                h_val = self.calculate_heuristic(idx, g_idx)
                new_f = self.f_value(state_space[idx].g, h_val, epsilon)
                new_open_list.append((new_f, idx))

            # 3. Restaurar las propiedades de la cola de prioridad O(N)
            heapq.heapify(new_open_list)
            OPEN = new_open_list
            
            # 4. Limpiar para la siguiente iteración
            INCONS.clear()
            CLOSED.clear()
            # (Opcional) Reconstruir OPEN completamente para actualizar 
            # las prioridades F(s) de los nodos que ya estaban adentro.

        # 6. Reconstruir la ruta y retornar

        if self._debug_mode and debug_markers.markers:
            # Añadimos un marcador especial para borrar líneas de metas anteriores
            delete_marker = Marker()
            delete_marker.action = Marker.DELETEALL
            debug_markers.markers.insert(0, delete_marker)
            
            self.debug_paths_pub.publish(debug_markers)

        if best_path_found:
            return self.reconstruct_path(s_idx, g_idx, state_space, start.header, x0, y0, res)
        else:
            return None


    # =================================================================
    # 6. FUNCIONES DE DEPURACIÓN (Opcional, para visualizar rutas subóptimas en RViz)
    # =================================================================

    def create_path_marker(self, path_cells: List[Tuple[int, int]], epsilon: float, 
                           marker_id: int, frame_id: str, x0: float, y0: float, res: float) -> Marker:
        marker = Marker()
        marker.header.frame_id = frame_id
        marker.header.stamp = self.get_clock().now().to_msg()
        marker.ns = "ara_eps_paths"
        marker.id = marker_id
        marker.type = Marker.LINE_STRIP
        marker.action = Marker.ADD
        
        # Grosor de la línea
        marker.scale.x = 0.01 
        
        # Lógica de Color (Rojo = subóptimo, Verde = óptimo)
        eps_start = self.get_parameter('ara_core.epsilon_start').value
        ratio = (epsilon - 1.0) / max((eps_start - 1.0), 0.01) # De 1.0 (Rojo) a 0.0 (Verde)
        
        marker.color = ColorRGBA()
        marker.color.r = max(0.0, min(1.0, float(ratio)))        # Más rojo si epsilon es alto
        marker.color.g = max(0.0, min(1.0, float(1.0 - ratio)))  # Más verde si epsilon se acerca a 1
        marker.color.b = 0.0
        marker.color.a = 0.8  # Ligeramente transparente

        # Convertir celdas a puntos 3D
        for ix, iy in path_cells:
            x, y = self.map_to_world(ix, iy, x0, y0, res)
            p = Point()
            p.x, p.y = x, y

            p.z = marker_id * 0.02
            marker.points.append(p)
            
        return marker

def main(args=None):
    rclpy.init(args=args)
    node = ARAPlannerNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()