# Team Hotel — Path Planning and Path Tracking (2602)

Implementación ROS 2 Jazzy para el robot diferencial Hotel. El proyecto conserva los componentes de simulación, localización, seguridad y control existentes, y añade la comparación A/B del assignment.

| Etapa | Baseline | Implementación Team Hotel |
|---|---|---|
| Path planning | `dijkstra_pp_2602_hotel` | `hybrid_astar_pp_2602_hotel` |
| Path tracking | `pure_pursuit_pt_2602_hotel` | `lqr_pt_2602_hotel` |

Los informes y datos medidos están en [`results/ASSIGNMENT_RESULTS_HOTEL.md`](results/ASSIGNMENT_RESULTS_HOTEL.md) y [`results/GAZEBO_CLOSED_LOOP_RESULTS_HOTEL.md`](results/GAZEBO_CLOSED_LOOP_RESULTS_HOTEL.md).

## Arquitectura

```text
/map + TF(map → base_link) + /goal_pose
                 │
                 ▼
  Dijkstra o Hybrid A* ── /planned_path ──► Pure Pursuit o LQR
                                                   │
                                              /cmd_vel_nav
                                                   │
  twist_mux → /cmd_vel_mux → AEB → /diffdrive_controller/cmd_vel
```

El robot usa `diff_drive_controller/DiffDriveController`, con radio de rueda de 0.05 m, separación de 0.44 m y límite de rueda de ±10 rad/s. El límite lineal físico derivado es 0.50 m/s; el launch de navegación ajusta ambos trackers a 0.45 m/s nominal y 2.0 rad/s máximo.

## Estructura y paquetes ROS 2

```text
src/
├── hotel_bringup/          lanzadores, twist_mux, AEB e interfaces de operación
├── hotel_description/      URDF/Xacro y configuración de robots
├── hotel_gazebo/           mundos, mapas y puente Gazebo–ROS
├── hotel_ekf/              EKF para la odometría
├── hotel_path_planner/     paquete ROS path_planner_2602_hotel
├── hotel_path_tracking/    paquete ROS path_tracker_2602_hotel
├── hotel_line_extractor/   extracción de líneas y lane keeping
├── hotel_wall_following/   seguimiento de pared
├── hotel_ttc_follow_the_gap/  TTC / Follow-the-Gap
├── hotel_control/          paquete de control
├── hotel_esc/              scripts y bag de identificación de sistema
├── vicon_v2/vicon/         interfaz Vicon
├── experiments/            benchmarks y análisis reproducibles
└── results/                CSV, informes y gráficas generadas
```

| Paquete ROS 2 | Rol | Ejecutables o launch relevantes |
|---|---|---|
| `path_planner_2602_hotel` | Planificación global sobre `OccupancyGrid`. | `dijkstra_pp_2602_hotel`, `hybrid_astar_pp_2602_hotel` |
| `path_tracker_2602_hotel` | Seguimiento de `nav_msgs/Path` para el diferencial. | `pure_pursuit_pt_2602_hotel`, `lqr_pt_2602_hotel` |
| `hotel_bringup` | Simulación, localización, multiplexado y selector. | `navigation_2602_hotel.launch.py`, `gz_spawn.launch.py` |
| `hotel_description`, `hotel_gazebo`, `hotel_ekf` | Descripción, simulación/mapas y estimación TF. | `ekf.launch.py`, localización AMCL/SLAM |
| `hotel_line_extractor`, `hotel_wall_following`, `hotel_ttc_follow_the_gap`, `hotel_control`, `vicon` | Capacidades adicionales del proyecto. | Launches de LKA, wall following y gap following. |

Los `setup.py` de planner y tracker también registran ejecutables de desarrollo (BIT*, ARA*, DWB y Stanley); no forman parte del selector del assignment.

## Interfaces ROS y TF

| Topic | Type | Dirección | Descripción |
|---|---|---|---|
| `/map` | `nav_msgs/OccupancyGrid` | localización → planner | Mapa estático usado por ambos planners. |
| `/goal_pose` | `geometry_msgs/PoseStamped` | RViz/usuario → planner | Goal en `map` o transformable a ese frame. |
| `/planned_path` | `nav_msgs/Path` | planner → tracker | Ruta calculada en el frame global. |
| `/cmd_vel_nav` | `geometry_msgs/TwistStamped` | tracker → `twist_mux` | Comando autónomo. |
| `/cmd_vel_mux` | `geometry_msgs/TwistStamped` | `twist_mux` → AEB | Salida multiplexada. |
| `/diffdrive_controller/cmd_vel` | `geometry_msgs/TwistStamped` | AEB → robot | Referencia del controlador diferencial. |
| `/diffdrive_controller/odom` | `nav_msgs/Odometry` | robot → EKF | Odometría de ruedas de la simulación. |
| `/scan` | `sensor_msgs/LaserScan` | Gazebo → ROS | LiDAR puenteado por `hotel_gazebo`. |
| `/imu` | `sensor_msgs/Imu` | Gazebo → ROS | IMU puenteada por `hotel_gazebo`. |

La cadena TF requerida es `map → odom → base_link`: `map → odom` viene de AMCL o `slam_toolbox`, y `odom → base_link` de odometría/EKF. `DiffDriveController` tiene `publish_tf: false`, por lo que debe iniciarse localización y `hotel_ekf` antes de enviar goals. Los planners recuperan el start con TF `map → base_link`; los trackers transforman el path al frame del robot.

```bash
ros2 run tf2_ros tf2_echo map base_link
```

## Path Planning

### Dijkstra

`dijkstra_pp_2602_hotel` es el baseline. Recibe `/map` y `/goal_pose`, consulta el start por TF y publica `nav_msgs/Path` en `/planned_path`. Busca sobre la grilla; por defecto usa vecindad de 8 conexiones y evita cortar esquinas. Ante mapa, TF o goal no válidos publica un path vacío, evitando que un tracker continúe una ruta obsoleta.

### Hybrid A*

`hybrid_astar_pp_2602_hotel` es el método asignado a Team Hotel. Su estado incluye `(x, y, theta, direction)`, por lo que puede respetar orientación final. Está adaptado a un diferencial: expande primitivas del unicycle (arcos con curvatura negativa, nula y positiva), reversa opcional y giro in-place. Cada arco se muestrea para validar colisiones contra el mapa inflado. El giro in-place tiene penalización explícita; no es una aproximación Ackermann. Publica `nav_msgs/Path` en `/planned_path`.

## Path Tracking

### Pure Pursuit

`pure_pursuit_pt_2602_hotel` es el baseline. Elige un punto de lookahead sobre `/planned_path`, calcula curvatura y publica `linear.x` y `angular.z` como `geometry_msgs/TwistStamped` en `/cmd_vel_nav`. Reduce velocidad cerca del goal y limita curvatura, velocidad y giro.

### LQR

`lqr_pt_2602_hotel` usa el error `e=[e_x,e_y,e_theta]` en el frame de referencia. Cada ciclo discretiza el modelo diferencial alrededor de la referencia, resuelve la ganancia LQR y aplica `delta_u=-K e`. Las matrices `Q` y `R` se ajustan con `q_*` y `r_*`; la salida saturada es `linear.x` y `angular.z` en `/cmd_vel_nav`. LQR y Pure Pursuit publican cero ante path vacío, TF no disponible o goal alcanzado.

## Compilación

Desde la raíz del workspace:

```bash
cd /home/lenovo/mrad_ws_2602_hotel
colcon build --symlink-install
source /opt/ros/jazzy/setup.bash
source install/setup.bash
```

Para compilar solo el stack del assignment:

```bash
colcon build --packages-select \
  path_planner_2602_hotel \
  path_tracker_2602_hotel \
  hotel_bringup \
  --symlink-install
source /opt/ros/jazzy/setup.bash
source install/setup.bash
```

Abra una terminal nueva por proceso ROS y repita los dos `source`.

## Simulación y localización

El launch diferencial inicia Gazebo, `robot_state_publisher`, puente de sensores, controladores, joystick, `twist_mux`, AEB y nodos auxiliares. El mundo por defecto es `walls_world2.sdf`; `gz_mode:=false` activa modo headless.

```bash
# Terminal 1: Gazebo, robot y cadena de comando
ros2 launch hotel_bringup gz_spawn.launch.py gz_mode:=false world:=walls_world2.sdf

# Terminal 2: odom → base_link
ros2 launch hotel_ekf ekf.launch.py

# Terminal 3: mapa y AMCL, map → odom
ros2 launch hotel_bringup amcl_localization.launch.py \
  map:=/home/lenovo/mrad_ws_2602_hotel/src/hotel_gazebo/maps/map2.yaml
```

Como alternativa existe el launch real de SLAM Toolbox:

```bash
ros2 launch hotel_bringup slam_localization.launch.py
```

No hay un launch de RViz en este repositorio. Si se inicia manualmente, use frame fijo `map` y compruebe TF. `walls_world2.sdf` y `map2.yaml` son el par de la validación cerrada. `exam1_world_obs.sdf` no se recomienda para medir colisiones: DART informó que no podía construir su colisión de malla.

## Ejecutar navegación y elegir métodos

Con simulación y `map → base_link` disponibles, el comando principal es:

```bash
ros2 launch hotel_bringup navigation_2602_hotel.launch.py
```

El launch inicia exactamente un planner y tracker. Argumentos reales: `planner:=dijkstra|hybrid_astar`, `tracker:=pure_pursuit|lqr`, `use_sim_time`, `scenario`, `trial` y `environment`.

```bash
# Dijkstra + Pure Pursuit (defaults)
ros2 launch hotel_bringup navigation_2602_hotel.launch.py planner:=dijkstra tracker:=pure_pursuit

# Hybrid A* + Pure Pursuit
ros2 launch hotel_bringup navigation_2602_hotel.launch.py planner:=hybrid_astar tracker:=pure_pursuit

# Dijkstra + LQR
ros2 launch hotel_bringup navigation_2602_hotel.launch.py planner:=dijkstra tracker:=lqr

# Hybrid A* + LQR
ros2 launch hotel_bringup navigation_2602_hotel.launch.py planner:=hybrid_astar tracker:=lqr
```

También se pueden ejecutar individualmente, sin duplicar publishers de `/planned_path` o `/cmd_vel_nav`:

```bash
ros2 run path_planner_2602_hotel dijkstra_pp_2602_hotel
ros2 run path_planner_2602_hotel hybrid_astar_pp_2602_hotel
ros2 run path_tracker_2602_hotel pure_pursuit_pt_2602_hotel
ros2 run path_tracker_2602_hotel lqr_pt_2602_hotel
```

`twist_mux.yaml` ya configura `cmd_vel_nav`; joystick y teclado tienen prioridad mayor, así que déjelos inactivos en pruebas autónomas.

## Enviar un goal

En RViz use **2D Goal Pose** con frame `map`. Para una ejecución reproducible, `/goal_pose` es `geometry_msgs/PoseStamped`:

```bash
ros2 topic pub --once /goal_pose geometry_msgs/msg/PoseStamped \
"{header: {frame_id: map}, pose: {position: {x: 2.623, y: 7.949, z: 0.0}, orientation: {z: 0.707107, w: 0.707107}}}"
```

La orientación importa especialmente con Hybrid A*, que verifica tolerancia de yaw final.

## Parámetros importantes

Estos son defaults de los nodos. El launch sobrescribe en Pure Pursuit `v_nominal=0.45`, `max_speed=0.50`, `max_omega=2.0`, `lookahead_L0=0.6` y `lookahead_min=0.3`.

### Hybrid A*

| Parámetro | Default | Efecto |
|---|---:|---|
| `xy_resolution` | 0.05 m | Resolución espacial de búsqueda. |
| `theta_resolution` | 0.261799 rad | Resolución angular (15°). |
| `motion_step` | 0.20 m | Longitud de las primitivas. |
| `max_curvature` | 1.6 1/m | Curvatura máxima. |
| `heuristic_weight` | 1.0 | Peso de heurística. |
| `inflate_radius` | 0.25 m | Inflación de obstáculos. |
| `turn_penalty` | 0.10 | Penalización por giro. |
| `rotation_penalty` | 0.12 | Coste de giro in-place. |
| `goal_position_tolerance` | 0.15 m | Tolerancia de posición. |
| `goal_yaw_tolerance` | 0.349066 rad | Tolerancia angular (20°). |

### LQR

| Parámetro | Default | Efecto |
|---|---:|---|
| `control_rate_hz` | 25.0 Hz | Frecuencia de control. |
| `v_nominal` | 0.45 m/s | Velocidad de referencia. |
| `max_speed` | 0.50 m/s | Límite lineal. |
| `max_omega` | 2.0 rad/s | Límite angular. |
| `goal_tolerance` | 0.25 m | Radio de llegada. |
| `lookahead_distance` | 0.25 m | Referencia adelantada. |
| `q_x` | 1.0 | Peso longitudinal de Q. |
| `q_lateral` | 6.0 | Peso lateral de Q. |
| `q_heading` | 3.0 | Peso angular de Q. |
| `r_linear` | 0.8 | Penalización lineal de R. |
| `r_angular` | 0.6 | Penalización angular de R. |

## Pruebas

```bash
colcon test --packages-select path_planner_2602_hotel path_tracker_2602_hotel hotel_bringup
colcon test-result --verbose
```

Las pruebas matemáticas específicas están en `hotel_path_planner/test/test_planning_core.py` y `hotel_path_tracking/test/test_tracking_core.py`: discretización, colisión de arcos, paths orientados, modelo/Riccati LQR, saturación y llegada a goal. Las 10 pruebas específicas pasan. Actualmente, la suite completa de `colcon test` no queda verde: al auditarla reportó 6 fallos de los linters `flake8`/`pep257` en código preexistente de los tres paquetes (incluye ejecutables baseline no usados por el selector). Los tests `test_copyright.py` están marcados `skip` hasta añadir cabeceras. Además, una colección global directa de `pytest` conserva el conflicto preexistente entre esos archivos baseline homónimos. No se modificaron esos baselines.

## Experimentos, benchmarks y resultados

Ejecute desde `src/`; los benchmarks no requieren Gazebo. El planner usa `OccupancyGrid` y el tracker una simulación cinemática offline de unicycle, no prueba física.

```bash
cd /home/lenovo/mrad_ws_2602_hotel/src
python3 experiments/run_planner_benchmark.py --trials 3
python3 experiments/run_hybrid_astar_sweep.py --trials 3
python3 experiments/run_tracker_benchmark.py --trials 3
python3 experiments/analyze_results.py
python3 experiments/analyze_gazebo_results.py
```

- `run_planner_benchmark.py`: comparación Dijkstra/Hybrid A* y paths/CSV.
- `run_hybrid_astar_sweep.py`: sweep de `theta_resolution`.
- `run_tracker_benchmark.py`: comparación y sweeps offline de trackers.
- `analyze_results.py`: informe y gráficas offline.
- `analyze_gazebo_results.py`: informe y gráficas a partir de CSV Gazebo.

Los resultados se guardan en `results/`: `planner_results.csv`, `tracker_trace.csv`, `tracker_results.csv`, `hybrid_astar_sweep.csv`, `lqr_sweep.csv`, `pure_pursuit_sweep.csv`, `gazebo_planner_results.csv`, `gazebo_tracker_results.csv`, `gazebo_tracker_trace.csv`, ambos informes Markdown y `results/plots/`. `analyze_results.py` regenera el informe offline; no lo ejecute si desea preservar el informe final validado tal como está.

### Resultados medidos destacados

En Gazebo, tramo recto `gazebo_map2_straight_repeated` (tres repeticiones por planner): Dijkstra tardó **16.881 ± 0.637 ms** y produjo **0.950 m**; Hybrid A* tardó **0.706 ± 0.108 ms** y produjo **1.000 m**. En tracking, una ejecución limpia por controlador midió CTE RMSE de **0.012474 m** para Pure Pursuit y **0.015519 m** para LQR; las tasas fueron 20.000 Hz y 20.002 Hz, respectivamente.

El tracking Gazebo fue una ejecución reset-controlada por controlador: no es una conclusión estadística general ni se extrapola a curvas u obstáculos.

## Troubleshooting

**Robot inmóvil.** Compruebe tracker, `/cmd_vel_nav`, prioridad de joystick/teclado y TF.

```bash
ros2 topic echo /cmd_vel_nav
ros2 topic info /diffdrive_controller/cmd_vel -v
ros2 run tf2_ros tf2_echo map base_link
```

**El planner no publica ruta.** Verifique `/map`, `map → base_link`, start/goal ocupados e `inflate_radius`.

```bash
ros2 topic echo /map --once
ros2 topic echo /planned_path --once
ros2 topic list
```

**El tracker publica stop.** Es seguro ante path vacío, TF ausente o goal alcanzado.

```bash
ros2 topic echo /planned_path --once
ros2 topic hz /cmd_vel_nav
ros2 run tf2_ros tf2_echo map base_link
```

## Limitaciones conocidas

- La clearance registrada es aproximación chamfer 8-conectada sobre el mapa, no medición física.
- No hay tres repeticiones Gazebo por tracker, pruebas Gazebo de curvas/pasillos/obstáculos, ni tracking sobre Hybrid A*.
- El modelo LQR no representa explícitamente latencia, deslizamiento, saturación conjunta de ruedas ni ruido de sensores.
- La inspección humana en RViz/GUI y contactos físicos no se validaron en el entorno headless de los resultados incluidos.
