# Team Hotel — Path Planning and Path Tracking Results

## 1. Sistema probado

- ROS 2 Jazzy (entorno de desarrollo detectado).
- Robot configurado: diferencial, `diff_drive_controller/DiffDriveController`, radio de rueda 0.05 m y separación 0.44 m.
- Límite físico derivado de la interfaz de rueda: 10 rad/s × 0.05 m = 0.50 m/s; el benchmark usa 0.45 m/s nominal y 2.0 rad/s máximo.
- Mapa de los benchmarks: `hotel_gazebo/maps/map_obs.pgm`, 0.05 m/celda, unknown tratado como obstáculo e inflación de 0.25 m.
- Frames configurados: `map`, `odom`, `base_link`; no existe `base_footprint` en la descripción diferencial.
- Interfaz de comando: `geometry_msgs/TwistStamped` en `/cmd_vel_nav`, integrado por `twist_mux` hacia `/cmd_vel_mux` y AEB hacia `/diffdrive_controller/cmd_vel`.
- Validación cerrada ejecutada: Gazebo headless con `walls_world2.sdf`, `diffdrive_controller`, puente `/scan`, EKF y AMCL con `hotel_gazebo/maps/map2.yaml`. `tf2_echo map base_link` resolvió `map → odom → base_link` después del arranque e inicialización de AMCL. Se verificaron `/map`, `/scan`, `/diffdrive_controller/odom`, `/planned_path` (por endpoint de nodo) y `/cmd_vel_nav`.

## 2. Métodos

Baseline: Dijkstra (`dijkstra_pp_2602_hotel`) y Pure Pursuit (`pure_pursuit_pt_2602_hotel`).

Hotel: Hybrid A* (`hybrid_astar_pp_2602_hotel`) y LQR (`lqr_pt_2602_hotel`).

## 3. Hybrid A* implementation

El estado es `(x, y, theta, direction)`. La parte `(x,y)` y la orientación se discretizan; las primitivas trasladan el modelo unicycle diferencial (`x_dot=v cos(theta)`, `y_dot=v sin(theta)`, `theta_dot=omega`) con curvaturas `{-k,0,+k}`. Cada arco se muestrea a mitad de una celda para validar colisión. Se permite giro in-place, porque un diferencial puede realizarlo; añade `rotation_penalty × |Δtheta|`, por lo que no es gratis. El costo suma longitud, giro, reversa/cambio de dirección si se habilitan y costo suave opcional. `h` es distancia euclídea y `f=g+w h`.

## 4. LQR implementation

El vector de error es `e=[e_x,e_y,e_theta]` (pose real menos referencia, en el frame de referencia); la entrada incremental es `delta_u=[v-v_ref, omega-omega_ref]`. El modelo continuo usado es `e_x_dot=-omega_ref e_y+delta_v`, `e_y_dot=omega_ref e_x+v_ref e_theta`, `e_theta_dot=delta_omega`; se discretiza por Euler y resuelve Riccati discreto para aplicar `delta_u=-K e`. Q por defecto diag(1,6,3) y R diag(0.8,0.6).

## A. OFFLINE RESULTS

### 5. Dijkstra vs Hybrid A*

**MEDIDO — benchmark offline sobre OccupancyGrid; no es tiempo de Gazebo.**

| scenario | method | success | n | length m | time ms | expanded | clearance m* | terminal yaw err rad |
|---|---|---|---|---|---|---|---|---|
| narrow | dijkstra | 3 | 3 | 8.926 | 310.05 | 21639 | 0.283 | 2.356 |
| narrow | hybrid_astar | 3 | 3 | 8.597 | 565.62 | 20428 | 0.262 | 0.000 |
| obstacle_manoeuvre | dijkstra | 3 | 3 | 8.294 | 220.44 | 14910 | 0.283 | 0.785 |
| obstacle_manoeuvre | hybrid_astar | 3 | 3 | 7.797 | 964.18 | 34842 | 0.262 | 0.000 |
| open | dijkstra | 3 | 3 | 5.649 | 153.34 | 10563 | 0.595 | 0.785 |
| open | hybrid_astar | 3 | 3 | 5.398 | 27.37 | 1037 | 0.595 | 0.000 |
| orientation_change | dijkstra | 3 | 3 | 5.794 | 176.90 | 11966 | 0.400 | 2.356 |
| orientation_change | hybrid_astar | 3 | 3 | 5.399 | 259.57 | 9584 | 0.400 | 0.000 |

*Clearance is an 8-connected chamfer approximation, calculated from the non-inflated obstacle map.

### 6. Pure Pursuit vs LQR

**CALCULADO/MEDIDO en simulación cinemática offline de unicycle, NO PROBADO en Gazebo o hardware.** Cada pareja usa exactamente el mismo `nav_msgs/Path` equivalente generado por Dijkstra, la misma perturbación inicial de 0.10 m lateral/0.10 rad y límites comunes.

| scenario | method | success | n | CTE RMSE m | CTE max m | time s | goal err m | mean |Δomega| | compute us |
|---|---|---|---|---|---|---|---|---|---|
| curves_obstacle | pure_pursuit | 0 | 6 | 0.458 | 0.666 | 5.08 | 5.500 | 0.0030 | 16.5 |
| curves_obstacle | lqr | 0 | 6 | 0.153 | 0.223 | 3.76 | 5.437 | 0.3171 | 2800.0 |
| curves_orientation | pure_pursuit | 3 | 3 | 0.245 | 0.804 | 14.32 | 0.243 | 0.0014 | 6.9 |
| curves_orientation | lqr | 3 | 3 | 0.056 | 0.199 | 11.00 | 0.242 | 0.0188 | 2806.4 |
| narrow | pure_pursuit | 6 | 6 | 0.358 | 0.862 | 25.92 | 0.243 | 0.0018 | 13.0 |
| narrow | lqr | 6 | 6 | 0.065 | 0.213 | 17.36 | 0.237 | 0.0496 | 2783.9 |
| straight_open | pure_pursuit | 6 | 6 | 0.272 | 0.866 | 13.40 | 0.238 | 0.0015 | 7.5 |
| straight_open | lqr | 6 | 6 | 0.057 | 0.204 | 10.72 | 0.237 | 0.0191 | 2785.0 |

Collision field in this offline run was checked against the inflated OccupancyGrid; it is not a physical collision sensor measurement.

### 7. Parameter sweeps

### Hybrid A* theta_resolution

| deg | success | time ms | expanded | length m |
|---|---|---|---|---|
| 45.0 | 1 | 694.89 | 25346 | 7.798 |
| 45.0 | 1 | 706.33 | 25346 | 7.798 |
| 45.0 | 1 | 667.78 | 25346 | 7.798 |
| 30.0 | 1 | 951.12 | 34842 | 7.797 |
| 30.0 | 1 | 950.57 | 34842 | 7.797 |
| 30.0 | 1 | 950.56 | 34842 | 7.797 |
| 22.5 | 1 | 1358.32 | 46659 | 7.795 |
| 22.5 | 1 | 1316.75 | 46659 | 7.795 |
| 22.5 | 1 | 1295.95 | 46659 | 7.795 |

### LQR q_lateral

| q_lateral | success | CTE RMSE m | max CTE m | mean |Δomega| |
|---|---|---|---|---|
| 2.0 | 1 | 0.062 | 0.185 | 0.0185 |
| 6.0 | 1 | 0.056 | 0.199 | 0.0188 |
| 12.0 | 1 | 0.052 | 0.203 | 0.0192 |

### Pure Pursuit lookahead_L0

| L0 m | success | CTE RMSE m | max CTE m | time s |
|---|---|---|---|---|
| 0.3 | 1 | 0.171 | 0.630 | 13.48 |
| 0.6 | 1 | 0.245 | 0.804 | 14.32 |
| 1.0 | 1 | 0.359 | 1.057 | 16.04 |
| 1.5 | 1 | 0.538 | 1.415 | 18.80 |

### 8. Hallazgos offline

### Hybrid A* fue mejor cuando…

- En `open`, Hybrid A* produjo 5.398 m frente a 5.649 m y cumplió yaw final (0.000 rad de error frente a 0.785 rad). Además tardó 27.37 ms frente a 153.34 ms en esta ejecución. Esto es consistente con que la heurística guió muy bien un espacio abierto; no se extrapola a los demás escenarios.

### Dijkstra fue mejor cuando…

- En `obstacle_manoeuvre`, Dijkstra tardó 220.44 ms y expandió 14910; Hybrid A* tardó 964.18 ms y expandió 34842. Hybrid redujo longitud de 8.294 a 7.797 m, pero ese ahorro geométrico no compensó el coste de cómputo si el heading terminal no era necesario.
- En `narrow`, Dijkstra también fue más rápido (310.05 vs 565.62 ms), mientras Hybrid A* redujo longitud de 8.926 a 8.597 m. Los clearances reportados son aproximados y no permiten afirmar una ventaja de seguridad.

### LQR fue mejor cuando…

- En `straight_open`, LQR redujo CTE RMSE de 0.272 a 0.057 m (79.1 %) y terminó en 10.72 s frente a 13.40 s.
- En `curves_orientation`, LQR redujo CTE RMSE de 0.245 a 0.056 m (77.1 %) y finalizó 3.32 s antes bajo el mismo modelo y path.
- En `narrow`, LQR redujo CTE RMSE de 0.358 a 0.065 m (81.8 %).

### Pure Pursuit fue mejor cuando…

- En los recorridos exitosos no superó a LQR en CTE bajo esta simulación. Sí tuvo un coste de cómputo menor: 6.9 us/ciclo frente a 2806.4 us/ciclo en `curves_orientation`, y comandos angulares mucho más suaves por la métrica media |Δomega| (0.0014 vs 0.0188).

### Caso de desventaja clara

- En `curves_obstacle` ambos controladores colisionaron con el mapa inflado tras la misma perturbación inicial y no llegaron al goal. LQR mantuvo menor CTE antes del fallo, pero **ninguno fue mejor en éxito**; evidencia que un path puramente grid y un tracker sin margen adicional no bastan para ese caso.

### Sweeps

- Al refinar `theta_resolution` de 45° a 22.5°, Hybrid A* aumentó el tiempo medio de 689.67 a 1323.68 ms y los estados de 25346 a 46659, con un cambio de longitud muy pequeño en la tabla. El compromiso observado es orientación más fina a cambio de búsqueda.
- En LQR, elevar `q_lateral` de 2.0 a 12.0 bajó RMSE de 0.062 a 0.052 m, pero subió CTE máximo de 0.185 a 0.203 m y |Δomega| de 0.0185 a 0.0192.
- En Pure Pursuit, `L0=0.3` m dio el menor RMSE (0.171 m); `L0=1.5` m lo elevó a 0.538 m pero redujo |Δomega| de 0.0025 a 0.0004.

## B. GAZEBO CLOSED-LOOP RESULTS

**MEDIDO — Gazebo headless + AMCL + EKF + `diff_drive_controller`; no son resultados offline.** Cada planner recibió el mismo `/goal_pose` en `map`, tomó la pose inicial de TF y publicó `nav_msgs/Path` en `/planned_path`. Para tracking se usó un path recién producido por Dijkstra, el mismo goal `(1.0, 0.0, 0)` y una reinicialización limpia de Gazebo/EKF/AMCL antes de cada controlador.

### Infraestructura validada

- `map → odom → base_link`: **SÍ**. AMCL quedó lifecycle `active`, aceptó `/initialpose`, y `tf2_echo map base_link` resolvió la cadena. El primer aviso de frame inexistente corresponde al corto intervalo anterior a la primera TF.
- `/map`: `nav_msgs/OccupancyGrid`, suministrado por `map_server` activo (`map2.yaml`, 402×403, 0.05 m/celda).
- `/scan`, `/diffdrive_controller/odom` y `/cmd_vel_nav`: activos; los nodos planner/tracker verificaron sus subscripciones/publicaciones. La salida `TwistStamped` pasó por el `twist_mux` existente.
- RViz con GUI y una inspección visual humana de los paths: **NO PROBADO** en este entorno headless. La validez de frame/orientaciones se comprobó mediante el `Path` publicado por el endpoint ROS y las métricas/poses registradas.

### Dijkstra vs Hybrid A*: mismo start TF y mismo goal, 3 repeticiones por planner

| method | success | planning time ms, mean ± sample sd | expanded | path length m | min clearance m* |
|---|---:|---:|---:|---:|---:|
| Dijkstra | 3/3 | 16.881 ± 0.637 | 1025 | 0.950 | 2.500 |
| Hybrid A* | 3/3 | 0.706 ± 0.108 | 20 | 1.000 | 2.500 |

*Clearance calculado como aproximación chamfer sobre el mapa; no es un sensor físico de distancia.

En este tramo recto y abierto, Hybrid A* fue aproximadamente 23.9 veces más rápido y expandió 51.2 veces menos estados, pero Dijkstra produjo un path 0.050 m (5.3 %) más corto. Esto **no** replica el mayor coste de Hybrid A* en obstáculos del benchmark offline: no hubo una maniobra compleja en esta prueba Gazebo, por lo que ambos resultados son compatibles y muestran dependencia del escenario.

### Dijkstra path → Pure Pursuit vs LQR: una repetición limpia por controlador

| method | success | CTE RMSE m | CTE MAE m | CTE max m | completion s | goal error m | mean |Δomega| rad/s | omega saturations |
|---|---|---:|---:|---:|---:|---:|---:|---:|
| Pure Pursuit | 1/1 | 0.012474 | 0.011756 | 0.019694 | 1.974357 | 0.260178 | 0.000326 | 0 |
| LQR | 1/1 | 0.015519 | 0.014847 | 0.020000 | 1.754421 | 0.236504 | 0.002078 | 0 |

En esta trayectoria recta real, Pure Pursuit redujo CTE RMSE en 19.6 % frente a LQR y también fue 6.4 veces más suave según `mean(|Δomega|)`. LQR terminó 0.220 s antes y con 0.024 m menos de error final. Ambos llegaron al goal sin saturación angular ni colisión observada. Por tanto, la ventaja offline de LQR de aproximadamente 77–82 % en CTE **no se mantuvo** en este único ensayo Gazebo recto: el resultado observado corresponde a la opción **D, Pure Pursuit funcionó mejor en CTE**. No se infiere el resultado para curvas, obstáculos o tres repeticiones.

### Frecuencia y tiempo real blando

Ambos controladores se configuraron a 20 Hz, con periodo disponible de 50 ms. `ros2 topic hz /cmd_vel_nav` midió 20.000 Hz para Pure Pursuit (periodos 0.049–0.051 s) y 20.002 Hz para LQR (periodos 0.049–0.051 s tras estabilización). Por ello, **ninguno incumplió** el deadline de 20 Hz. El coste LQR offline de aproximadamente 2806 µs/ciclo (2.806 ms) ocupa aproximadamente 5.6 % de un periodo de 50 ms; aun sin afirmar que sea idéntico en Gazebo, la tasa medida demuestra que cumple holgadamente el objetivo de tiempo real blando en este escenario.

### Seguridad/fallos observados

- No se observaron saturaciones de omega, pérdidas de path ni fallos de TF durante los dos recorridos medidos.
- Se detectó y corrigió durante la validación un problema de banco de pruebas: procesos EKF/`twist_mux` huérfanos de intentos anteriores publicaban TF duplicada y conservaban una pose vieja. Tras detenerlos, quedó una sola instancia de cada nodo y `map → base_link` volvió a la pose de reinicio. No fue un bug de Hybrid A*, LQR ni de la cinemática diferencial.
- Se detectó y corrigió un bug real de instrumentación LQR: tras alcanzar el goal el nodo seguía agregando muestras de stop al trace ya terminado. El nodo ahora desactiva el path al cerrar el run. La traza bruta de esta ejecución se conserva; las gráficas Gazebo usan únicamente las 39 (Pure Pursuit) y 36 (LQR) muestras delimitadas por sus resúmenes medidos.
- El mundo `exam1_world_obs.sdf` no se usó para métricas: Gazebo/DART informó que no podía construir la colisión de malla `maze_collision`. Se usó `walls_world2.sdf`, cuyas paredes/cajas sí son geometría SDF nativa. No se midieron impactos con un sensor de colisión.

Las gráficas exclusivas de esta sección son `results/plots/gazebo_planner_comparison.png`, `gazebo_tracker_trajectory_xy.png`, `gazebo_tracker_cte_vs_time.png`, `gazebo_tracker_omega_vs_time.png` y `gazebo_tracker_summary.png`. Los CSV fuente están en `results/gazebo_planner_results.csv`, `gazebo_tracker_trace.csv` y `gazebo_tracker_results.csv`.

## C. NOT TESTED / LIMITACIONES ABIERTAS

- **NO PROBADO:** tres repeticiones reset-controladas por tracker (solo una por controlador), curvas Gazebo, pasillos, obstáculos que exijan maniobra, sweep LQR en Gazebo, sweep Pure Pursuit en Gazebo y comparación de tracking sobre Hybrid A*.
- **NO PROBADO:** inspección visual humana en RViz/GUI y colisiones físicas medidas. La ausencia de clipping/corner cutting en los dos trayectos rectos se obtuvo por la traza TF y el estado de éxito, no por un detector de contacto.
- La distancia de clearance es aproximada por chamfer 8-conectado, no una distancia euclídea exacta.
- El modelo LQR no incluye latencia, deslizamiento, saturación conjunta de ruedas ni ruido de sensores.
- Software: `colcon build` de los tres paquetes finales y 10 pruebas matemáticas específicas pasaron. La colección global de `pytest` sigue bloqueada por dos tests baseline homónimos `test_copyright.py` (uno en cada paquete), un conflicto de nombres preexistente que no se modificó para evitar alterar esos baselines.

## 10. Recomendación

Para mapas complejos y requisito de yaw final, Hybrid A* está justificado por los resultados offline: paths más cortos y yaw terminal nulo, aceptando mayor coste de búsqueda en `obstacle_manoeuvre` y `narrow`. En el único tramo recto Gazebo medido, Dijkstra fue 5.3 % más corto y Hybrid A* fue 23.9 veces más rápido; no hay evidencia Gazebo suficiente para recomendar uno universalmente en obstáculos. Para tracking recto y 20 Hz, el resultado medido recomienda Pure Pursuit por menor CTE y comando angular más suave; LQR queda recomendado cuando se priorice su menor tiempo de llegada o cuando ensayos futuros en curvas/perturbaciones confirmen la ventaja offline. Ambos cumplen el deadline de 50 ms en el caso probado.

## Presentation-ready conclusions

- **MEDIDO Gazebo — Hybrid A***: en el tramo abierto fue 23.9× más rápido que Dijkstra (0.706 vs 16.881 ms), pero su path fue 5.3 % más largo (1.000 vs 0.950 m).
- **MEDIDO offline — Hybrid A***: en `obstacle_manoeuvre` redujo longitud de 8.294 a 7.797 m, a costa de 964.18 vs 220.44 ms; el trade-off se invierte según el escenario.
- **MEDIDO Gazebo — Pure Pursuit**: en el path Dijkstra recto redujo CTE RMSE de 0.015519 a 0.012474 m frente a LQR (19.6 %), con 6.4× menor `mean(|Δomega|)`.
- **MEDIDO Gazebo — LQR**: completó 0.220 s antes (1.754 vs 1.974 s) y con menor error final (0.2365 vs 0.2602 m), sin saturación de omega.
- **Frecuencia**: ambos publicaron `/cmd_vel_nav` a ≈20.0 Hz; el periodo 50 ms se cumplió pese a los 2.806 ms/ciclo LQR offline.
- **NO extrapolar**: Gazebo tiene solo un trial limpio de tracking por controlador y una trayectoria recta; no afirma superioridad en curvas u obstáculos.
- Para diapositivas, priorizar `gazebo_tracker_summary.png` y `gazebo_tracker_cte_vs_time.png` para la diferencia real de tracking; `gazebo_planner_comparison.png` para el caso abierto; y los gráficos offline `planner_paths_obstacle_manoeuvre.png` y `hybrid_astar_theta_sweep.png` para la complejidad y sensibilidad no cubiertas en Gazebo.
