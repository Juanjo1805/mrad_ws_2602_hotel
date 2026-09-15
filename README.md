# Team Hotel — Path Planning, Tracking y evasión reactiva

Entrega ROS 2 Jazzy + Gazebo Harmonic para robot diferencial: localización, planificación global, seguimiento, seguridad y evasión local de obstáculos no mapeados.

## Arquitectura general

```text
/map -> AMCL -> map -> odom
wheel odometry + IMU -> hotel_ekf -> odom -> base_link

waypoints -> Dijkstra | Hybrid A* -> /planned_path
                                      -> Pure Pursuit | LQR | Adaptive Pure Pursuit
                                                                 -> /cmd_vel_nav --+
Follow-The-Gap reactivo ----------------------------------------> /cmd_vel_gap --+-> twist_mux
                                                                                   -> /cmd_vel_mux -> AEB
                                                                                   -> /diffdrive_controller/cmd_vel -> robot
```

Hybrid A* produce la ruta global; Adaptive Pure Pursuit la sigue normalmente. El FTG reactivo no replantea: solo toma control cuando un obstáculo bloquea el corredor futuro, luego reengancha por delante al path original. AEB está después del mux y protege ambos mandos.

## Paquetes ROS 2

| Paquete | Rol | Ejecutables/launch relevantes |
|---|---|---|
| `path_planner_2602_hotel` (`hotel_path_planner`) | Planificación y misión por waypoints. | `dijkstra_pp_2602_hotel`, `hybrid_astar_pp_2602_hotel`, `fixed_waypoint_planner`, `editable_waypoint_manager` |
| `path_tracker_2602_hotel` (`hotel_path_tracking`) | Seguimiento diferencial. | `pure_pursuit_pt_2602_hotel`, `lqr_pt_2602_hotel`, `adaptive_pure_pursuit`, `publish_initial_pose` |
| `hotel_bringup` | Gazebo, AMCL, controladores, mux y AEB. | `gz_spawn.launch.py`, `amcl_localization.launch.py`, `navigation_2602_hotel.launch.py` |
| `hotel_description` | URDF/Xacro y límites físicos. | `diffdrive_urdf/robot.urdf.xacro` |
| `hotel_ttc_follow_the_gap` | FTG y supervisor reactivo. | `ttc_gap_finder`, `ttc_control`, `reactive_avoidance_supervisor` |
| `hotel_ekf` | Fusión local y TF. | `ekf.launch.py`, `ekf_node` |

## Path Planning

### Dijkstra

Busca el coste acumulado mínimo sobre el `OccupancyGrid`, con vecindad de 8 conexiones y prevención de corner cutting. Recibe mapa, goal y start por TF; publica `/planned_path` como `nav_msgs/msg/Path` en `map`.

### Hybrid A*

`hybrid_astar_pp_2602_hotel` planifica posición, orientación y dirección mediante primitivas de unicycle. Valida arcos contra el mapa inflado, permite reversa/giro in-place según parámetros y respeta orientación final. Es el planner principal de la demostración porque genera rutas coherentes con un diferencial.

La misión cerrada usa [`fixed_waypoints.yaml`](hotel_path_planner/config/fixed_waypoints.yaml), frame `map`, con **46 waypoints**. `fixed_waypoint_planner` concatena tramos y publica un único `/planned_path` denso.

## Path Tracking

### Pure Pursuit y LQR

Pure Pursuit selecciona closest point y lookahead, calcula curvatura y publica `v, omega`. Mantiene progreso monótono, rechaza targets detrás del robot, valida frames y evita terminar prematuramente en una ruta cerrada. LQR usa el error unicycle/diferencial `e=[e_x,e_y,e_theta]`, realimentación LQR y saturaciones físicas.

### Adaptive Pure Pursuit

Es el tracker nominal. Combina lookahead dinámico, velocidad dependiente de geometría y preview de curvatura:

```text
Ld = clamp((lookahead_base + lookahead_speed_gain*|v_cmd[k-1]|) /
           (1 + lookahead_curvature_gain*|kappa_preview|), Ld_min, Ld_max)
```

El target de velocidad toma el mínimo de techos por curvatura actual/futura, capacidad angular, error lateral, heading y rueda, y se rate-limita.

| Parámetro actual | Valor |
|---|---:|
| `min_linear_velocity`, `nominal_linear_velocity`, `max_linear_velocity` | 0.25, 0.90, 1.00 m/s |
| `max_angular_velocity` | 4.00 rad/s |
| `lookahead_min`, `lookahead_base`, `lookahead_max` | 0.30, 0.60, 1.20 m |
| `lookahead_speed_gain`, `lookahead_curvature_gain` | 0.60, 0.60 |
| `curvature_preview_distance` | 1.20 m |
| `acceleration_limit`, `deceleration_limit` | 0.70, 1.40 m/s² |
| `wheel_max_angular_velocity` | 20 rad/s |

Diagnósticos publicados por el tracker:

```text
/path_tracking/current_speed              /path_tracking/target_speed
/path_tracking/speed_limit_curvature      /path_tracking/speed_limit_preview
/path_tracking/speed_limit_omega          /path_tracking/speed_limit_lateral_error
/path_tracking/lookahead_distance         /path_tracking/curvature
/path_tracking/future_curvature           /path_tracking/lateral_error
/path_tracking/heading_error              /path_tracking/path_progress
/path_tracking/closest_index
```

## Límites, localización y TF

El robot tiene radio de rueda **0.05 m**, separación **0.44 m** y límites de rueda **±20 rad/s**. El límite lineal teórico es `20*0.05=1.00 m/s`; el angular cinemático aproximado es `1/0.22=4.55 rad/s`.

La TF es `map -> odom -> base_link`: AMCL publica `map -> odom`; `hotel_ekf` publica el único `odom -> base_link` (`frames.publish_tf: true`). El EKF estima `[x,y,yaw,v,omega]`, fusiona velocidad/yaw-rate de `/diffdrive_controller/odom` y `angular_velocity.z` de `/imu`. `diffdrive_controller` tiene `publish_tf: false` y `enable_odom_tf: false`.

### AMCL y mapa

AMCL se carga con [`map_nuevo.yaml`](hotel_gazebo/maps/map_nuevo.yaml), mediante `amcl_localization.launch.py` y `map:=...`. La pose inicial se publica en `/initialpose` (`geometry_msgs/msg/PoseWithCovarianceStamped`).

### Spawn inicial del robot

`gz_spawn.launch.py` usa por defecto `exam1_world_obs.sdf` y spawn físico:

```text
x=-15.4 m, y=0.0 m, z=0.5 m, roll=0, pitch=0, yaw=1.57 rad
```

El spawn físico de Gazebo y la pose global AMCL no son equivalentes; deben ser coherentes para no introducir offset, mal tracking o activaciones AEB.

En la secuencia manual validada, después de que AMCL esté activo se publica la pose global `map` `(0, 0, 0 rad)` con el ejecutable del workspace:

```bash
ros2 run path_tracker_2602_hotel publish_initial_pose --ros-args \
  -p use_sim_time:=true -p x:=0.0 -p y:=0.0 -p yaw:=0.0 -p frame_id:=map
```

No sustituya los valores de spawn de Gazebo por esta pose: el origen físico del mundo y el origen del mapa son referencias distintas.

## Evasión reactiva de obstáculos no mapeados

Implementación: [`reactive_core.py`](hotel_ttc_follow_the_gap/hotel_ttc_follow_the_gap/reactive_core.py), [`reactive_avoidance_supervisor.py`](hotel_ttc_follow_the_gap/hotel_ttc_follow_the_gap/reactive_avoidance_supervisor.py) y [`reactive_avoidance.yaml`](hotel_ttc_follow_the_gap/config/reactive_avoidance.yaml).

```text
TRACKING -> AVOIDING -> REJOINING -> TRACKING
```

- **TRACKING:** Adaptive PP conserva hasta 1.00 m/s.
- **Detección:** usa LiDAR y corredor del tramo futuro del path, no el mínimo global del scan. Corredor = semiancho del chasis 0.20 m + margen 0.15 m = **0.35 m**. Exige 3 retornos, distancia ≤2.20 m y persistencia 0.30 s.
- **Preview:** `max(base + gain*v, v²/(2*deceleration) + latency*v + front_extent + safety_margin)`, entre 1.00 y 2.50 m; a 1.0 m/s resulta 1.65 m.
- **AVOIDING:** FTG publica `/cmd_vel_gap` (`TwistStamped`) hasta 0.68 m/s y 2.00 rad/s. El mux da prioridad 150 frente a 100 de `/cmd_vel_nav`.
- **REJOINING:** tras corredor libre 0.60 s y mínimo 0.80 s evitando, Adaptive PP recupera mando a máximo 0.68 m/s. Rejoin queda 0.70 m adelante, sin retroceder índice. Vuelve a TRACKING tras 0.80 s con `|cte|<0.15 m` y `|heading|<0.22 rad`.

FTG usa FOV 120°, preprocessing, bubble, gaps con ancho físico mínimo 0.55 m y sesgo opcional al path. La anchura es `2*range_representativo*sin(anchura_angular/2)`.

**Corrección de ingeniería:** la versión inicial expandía bubble alrededor de cada retorno <2.20 m; paredes/retornos superpuestos borraron 241/241 beams, dejando 0 candidates, 0 gaps válidos, ancho y target en cero. Ahora la bubble se expande únicamente alrededor del retorno bloqueante más cercano. El test de caja frontal obtiene `ftg_valid_gap=true`, ancho positivo y ángulo no nulo.

## Arbitraje y seguridad

```text
/cmd_vel_nav + /cmd_vel_gap -> twist_mux -> /cmd_vel_mux -> AEB -> /diffdrive_controller/cmd_vel
```

FTG nunca bypassa AEB. No se redujeron TTC, márgenes, thresholds ni sector de seguridad para habilitar la evasión.

## Topics importantes

| Topic | Tipo | Publisher | Propósito |
|---|---|---|---|
| `/scan` | `sensor_msgs/msg/LaserScan` | bridge Gazebo | LiDAR |
| `/imu` | `sensor_msgs/msg/Imu` | bridge Gazebo | IMU |
| `/map` | `nav_msgs/msg/OccupancyGrid` | map server | Mapa global |
| `/amcl_pose` | `geometry_msgs/msg/PoseWithCovarianceStamped` | AMCL | Estimación global |
| `/initialpose` | `geometry_msgs/msg/PoseWithCovarianceStamped` | usuario / `publish_initial_pose` | Inicialización de AMCL |
| `/tf`, `/tf_static` | `tf2_msgs/msg/TFMessage` | AMCL, EKF, robot state publisher | Cadena de transformaciones |
| `/planned_path` | `nav_msgs/msg/Path` | planner | Ruta global |
| `/cmd_vel_nav` | `geometry_msgs/msg/TwistStamped` | tracker | Mando nominal |
| `/cmd_vel_gap` | `geometry_msgs/msg/TwistStamped` | supervisor reactivo | Mando FTG durante AVOIDING |
| `/cmd_vel_mux` | `geometry_msgs/msg/TwistStamped` | `twist_mux` | Mando arbitrado |
| `/diffdrive_controller/cmd_vel` | `geometry_msgs/msg/TwistStamped` | AEB | Mando seguro al robot |
| `/ekf/odometry`, `/diffdrive_controller/odom` | `nav_msgs/msg/Odometry` | `hotel_ekf`/DiffDriveController | Estado fusionado y odometría de rueda |
| `/reactive_avoidance/state` | `std_msgs/msg/String` | supervisor | Estado reactivo |
| `/reactive_avoidance/active`, `/blocking_obstacle`, `/unmapped_obstacle`, `/path_clear`, `/ftg_valid_gap` | `std_msgs/msg/Bool` | supervisor | Decisiones |
| `/reactive_avoidance/obstacle_distance`, `/preview_distance`, `/corridor_width`, `/rejoin_distance`, `/rejoin_heading_error`, `/ftg_target_angle`, `/ftg_gap_width`, `/rejoin_speed_limit` | `std_msgs/msg/Float32` | supervisor | Diagnóstico |
| `/reactive_avoidance/rejoin_index`, `/ftg_candidate_gap_count`, `/ftg_valid_gap_count` | `std_msgs/msg/Int32` | supervisor | Progreso/pipeline |
| `/reactive_avoidance/ftg_gap_details` | `std_msgs/msg/String` | supervisor | Candidates y rechazos |
| `/reactive_avoidance/markers` | `visualization_msgs/msg/MarkerArray` | supervisor | Corredor, hits, target, rejoin, estado |

Los diagnósticos de seguimiento y reactivos anteriores se publican con los tipos estándar indicados en la tabla. Para evitar ambigüedad al grabar o monitorizar, los nombres completos son:

```text
/path_tracking/current_speed              /path_tracking/target_speed
/path_tracking/speed_limit_curvature      /path_tracking/speed_limit_preview
/path_tracking/speed_limit_omega          /path_tracking/speed_limit_lateral_error
/path_tracking/lookahead_distance         /path_tracking/curvature
/path_tracking/future_curvature           /path_tracking/lateral_error
/path_tracking/heading_error              /path_tracking/path_progress
/path_tracking/closest_index

/reactive_avoidance/state                 /reactive_avoidance/active
/reactive_avoidance/blocking_obstacle     /reactive_avoidance/unmapped_obstacle
/reactive_avoidance/obstacle_distance     /reactive_avoidance/path_clear
/reactive_avoidance/preview_distance      /reactive_avoidance/corridor_width
/reactive_avoidance/rejoin_index          /reactive_avoidance/rejoin_distance
/reactive_avoidance/rejoin_heading_error  /reactive_avoidance/rejoin_speed_limit
/reactive_avoidance/ftg_target_angle      /reactive_avoidance/ftg_gap_width
/reactive_avoidance/ftg_valid_gap         /reactive_avoidance/ftg_candidate_gap_count
/reactive_avoidance/ftg_valid_gap_count   /reactive_avoidance/ftg_gap_details
/reactive_avoidance/markers
```

## Compilación

```bash
cd /home/lenovo/mrad_ws_2602_hotel
source /opt/ros/jazzy/setup.bash
colcon build --symlink-install
source install/setup.bash
```

Para una compilación selectiva del stack documentado:

```bash
colcon build --symlink-install --packages-select \
  path_planner_2602_hotel path_tracker_2602_hotel hotel_bringup hotel_ttc_follow_the_gap
source install/setup.bash
```

## Ejecución paso a paso

Terminal 1:

```bash
ros2 launch hotel_bringup gz_spawn.launch.py \
  world:=exam1_world_obs.sdf x_pose:=-15.4 y_pose:=0.0 z_pose:=0.5 yaw:=1.57
```

Este launch inicia Gazebo, el robot, controladores, puente de sensores, `twist_mux`, AEB y, por defecto, `hotel_ekf` (`start_ekf:=true`). Espere a que controladores, `/scan`, `/ekf/odometry` y TF estén disponibles antes de continuar.

Terminal 2:

```bash
ros2 launch hotel_bringup amcl_localization.launch.py \
  map:=/home/lenovo/mrad_ws_2602_hotel/src/hotel_gazebo/maps/map_nuevo.yaml
```

Terminal 3 — posición inicial de AMCL (ejecútelo cuando AMCL ya esté activo):

```bash
ros2 run path_tracker_2602_hotel publish_initial_pose --ros-args \
  -p use_sim_time:=true \
  -p x:=0.0 \
  -p y:=0.0 \
  -p yaw:=0.0 \
  -p frame_id:=map
```

Terminal 4 — navegación:

```bash
ros2 launch hotel_bringup navigation_2602_hotel.launch.py \
  planner:=hybrid_astar tracker:=adaptive_pure_pursuit reactive_avoidance:=true \
  mission_mode:=fixed_waypoints laps:=2 use_sim_time:=true \
  waypoints_file:=/home/lenovo/mrad_ws_2602_hotel/src/hotel_path_planner/config/fixed_waypoints.yaml \
  adaptive_params_file:=/home/lenovo/mrad_ws_2602_hotel/src/hotel_path_tracking/config/adaptive_pure_pursuit.yaml \
  reactive_params_file:=/home/lenovo/mrad_ws_2602_hotel/src/hotel_ttc_follow_the_gap/config/reactive_avoidance.yaml
```

Sin evasión: `reactive_avoidance:=false`. Valores válidos: planners `dijkstra|hybrid_astar`; trackers `pure_pursuit|lqr|adaptive_pure_pursuit`; misión `goal|fixed_waypoints`.

| Componente | Valores admitidos por `navigation_2602_hotel.launch.py` |
|---|---|
| Planner | `dijkstra`, `hybrid_astar` |
| Tracker | `pure_pursuit`, `lqr`, `adaptive_pure_pursuit` |
| Misión | `goal`, `fixed_waypoints` |
| Evasión reactiva | `reactive_avoidance:=true` o `false` |

## Monitoreo y RViz

```bash
ros2 topic echo /reactive_avoidance/state
ros2 topic echo /reactive_avoidance/ftg_valid_gap
ros2 topic echo /reactive_avoidance/ftg_gap_width
ros2 topic echo /reactive_avoidance/ftg_target_angle
ros2 topic echo /reactive_avoidance/ftg_gap_details
ros2 topic echo /cmd_vel_gap
ros2 topic echo /cmd_vel_mux
ros2 topic echo /path_tracking/current_speed
ros2 topic echo /path_tracking/path_progress
ros2 topic hz /cmd_vel_nav
ros2 topic hz /cmd_vel_gap
ros2 run tf2_ros tf2_echo map base_link
ros2 run tf2_ros tf2_echo map odom
ros2 run tf2_ros tf2_echo odom base_link
```

En RViz use Fixed Frame `map` y añada Map, LaserScan, Path, TF, RobotModel y `/reactive_avoidance/markers`.

Los markers muestran el corredor futuro evaluado, los retornos/obstáculo bloqueante, el gap y target del FTG, el punto de reenganche y una etiqueta con el estado reactivo. Esto permite observar la transición `TRACKING -> AVOIDING -> REJOINING -> TRACKING` sin inferirla únicamente desde los comandos de velocidad.

## Validación y resultados

```bash
pytest -q \
  src/hotel_path_planner/test/test_planning_core.py \
  src/hotel_path_tracking/test/test_tracking_core.py \
  src/hotel_ttc_follow_the_gap/test/test_reactive_core.py
```

En el estado documentado, las pruebas matemáticas de planificación/tracking dan **34 passed** y la suite reactiva da **13 passed**. Esta última incluye caja frontal, comparación contra FTG original, pipeline de gaps, persistencia/rejoin y límites cinemáticos.

La comparación manual está en [`../optimization_results/comparison_run001_vs_run002.md`](../optimization_results/comparison_run001_vs_run002.md): Run 001: 755.031 s, 0.360 m/s, RMSE 0.102 m; Run 002: 455.640 s, 0.622 m/s, RMSE 0.063 m; mejora end-to-end 39.653%. Los paths/waypoints no fueron idénticos, por lo que no es atribución causal pura solo al controlador.

Los informes históricos adicionales se conservan en [`results/`](results/); los resultados de planificación/tracking se deben interpretar con la configuración de mapa, spawn y waypoints registrada para cada ejecución.

### Benchmarks históricos conservados

El directorio [`experiments/`](experiments/) conserva los scripts offline de comparación de planners, barrido de Hybrid A*, benchmark de trackers y análisis de CSV. Se ejecutan desde `src/` y regeneran archivos bajo `results/`, por lo que no deben ejecutarse si se desea preservar un informe final concreto:

```bash
cd /home/lenovo/mrad_ws_2602_hotel/src
python3 experiments/run_planner_benchmark.py --trials 3
python3 experiments/run_hybrid_astar_sweep.py --trials 3
python3 experiments/run_tracker_benchmark.py --trials 3
python3 experiments/analyze_results.py
python3 experiments/analyze_gazebo_results.py
```

## Diagnóstico rápido

- **No se publica `/planned_path`:** compruebe `/map`, `map -> base_link`, `/initialpose` y que start/goal no estén ocupados o fuera del mapa.
- **El robot no avanza:** inspeccione `/cmd_vel_nav`, `/cmd_vel_mux`, `/diffdrive_controller/cmd_vel` y que joystick/teclado no estén venciendo la prioridad autónoma del mux.
- **FTG no toma una salida válida:** observe `ftg_valid_gap`, los contadores de gaps y `ftg_gap_details`; AEB sigue deteniendo el robot si el mando reactivo no encuentra un paso seguro.

## Problemas corregidos y decisiones

- `_parameters()` colisionaba con `rclpy.Node._parameters`; ahora es `_declare_parameters()`. También se retiró la redeclaración de `use_sim_time`.
- La bubble múltiple eliminaba el FOV; se limita al retorno bloqueante más cercano.
- Spawn físico y AMCL deben ser coherentes.
- Hybrid A* sigue siendo global, FTG solo local, Adaptive PP nominal y AEB nunca se bypassa. La velocidad nominal no se redujo globalmente para incorporar evasión.
