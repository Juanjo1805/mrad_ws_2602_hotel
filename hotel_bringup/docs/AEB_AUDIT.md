# Auditoría usada para implementar el AEB RC

Fecha: 2026-09-26. Este documento conserva el contexto de la auditoría inicial.
**Corrección posterior:** Hotel es ahora la fuente de verdad. La copia R es sólo
respaldo/reference y no se modifica. En la arquitectura vigente, mux y AEB corren
en el PC; sólo ybeb y el driver LiDAR corren en Raspberry. Los comandos históricos
al final son evidencia de la fase anterior, no instrucciones de operación actual.
Ver [README vigente](../README.md).

## Referencias e historial

- H: `/home/lenovo/mrad_ws_2602_hotel/src` (raíz del Git de Hotel).
- R: `/home/lenovo/proyecto_rc/raspberry_pi_5_backup/ros2_ws_2602/src/ybeb_2602_zulu`.
- V: `/home/lenovo/proyecto_rc/ros2_ws_2601/src`.
- AEB: `H/hotel_bringup/hotel_bringup/aeb_node.py`.
- Preprocesamiento: `H/hotel_bringup/hotel_bringup/lidar_data.py`.
- Configuraciones: `H/hotel_bringup/config/{twist_mux,joystick}.yaml`.
- Launch: `H/hotel_bringup/launch/hotel_joy.launch.py`, `gz_spawn.launch.py`,
  `ackermann.launch.py`, `ackerman_gz.launch.py`.
- Git `16cd27f` (2026-08-18, Creacion del repo): lógica inicial.
- Git `0557bc7` (2026-09-10, Primeras 2 vueltas exitosas con Hybrid y Pure):
  cambio de fin de archivo del AEB, sin nueva decisión de frenado.
- Git `61d1d45` (2026-09-14, Mejora de velocidad): añade diagnósticos del AEB;
  conserva la decisión de frenado.

## Contrato y algoritmo Hotel

| Dirección | Tópico | Tipo |
|---|---|---|
| Entrada | `/cmd_vel_mux` | `geometry_msgs/msg/TwistStamped` |
| Entrada | `/lidar/vctrl` | `std_msgs/msg/Float32` |
| Entrada | `/lidar/d_min` | `std_msgs/msg/Float32` |
| Entrada | `/lidar/front_scan` | `sensor_msgs/msg/LaserScan` |
| Salida | `/cmd_vel_out` | `geometry_msgs/msg/TwistStamped` |
| Diagnóstico | `/dist_min` | `geometry_msgs/msg/Twist` |
| Diagnóstico | `/aeb/active`, `/aeb/command_blocked` | `std_msgs/msg/Bool` |
| Diagnóstico | `/aeb/ttc`, `/aeb/critical_distance` | `std_msgs/msg/Float32` |

Suscripciones originales con QoS fiable de profundidad 10. No existe timer de
seguridad: el paso de comandos depende de callbacks de comandos; frenado y
diagnósticos dependen de callbacks de scan. No hay frecuencia periódica garantizada.

Parámetros originales: `ttc_threshold=0.45 s`, `secure_min_distance=0.6 m`,
`robot_radius=0.45 m`, `min_speed=0.05 m/s`, `brake_gain=1.5`, `brake_max=2.0`.
Los launch examinados usan esos valores predeterminados; no hay YAML AEB dedicado.

Fórmula: `v_closing = v_ctrl*cos(theta)`, sólo rayos `v_closing > 0.1`;
`TTC=min((r-robot_radius)/v_closing)`. Si cualquier distancia efectiva es <=0,
devuelve 0. Si `v_ctrl<0.05` o no hay scan, devuelve infinito. Por ello la reversa
y la velocidad cero quedan fuera del TTC original.

Bloqueo: `(d_min<=1.4 AND TTC<0.45) OR d_min<=0.45`. La distancia 1.4 está
codificada directamente. Si hay bloqueo y `v_ctrl>0.1`, empieza contracorriente:
`linear.x=-min(v_ctrl*1.5,2.0)`, o -2.0 estando dentro del radio. Conserva steering.
Termina contracorriente al bajar `v_ctrl<=0.05` y publica cero. El bloqueo de
avance se activa con bloqueo y `d_min<0.6`, y se libera al llegar a `d_min>=0.6`.
No tiene histéresis TTC ni timeout de ningún dato. Durante frenado ignora callbacks
de comandos; sin frenado transmite comandos. Perder scan puede dejar pasar una
orden sin evidencia nueva o mantener el último frenado; perder comando tampoco
garantiza neutral. El hardware original conserva su última orden indefinidamente.

El AEB no valida directamente NaN, infinito, range_min/max ni scans vacíos. Un
NaN recibido directamente puede contaminar el mínimo y hacer falsas las
comparaciones. `lidar_data.py` filtra previamente rangos: acepta sólo finitos
estrictamente entre range_min/max y sustituye el resto por infinito.

`lidar_data.py` recorta el frente a ±20° y estima velocidad en ±30°. Usa diferencias
radiales entre scans con dt en (0.0001,0.5], al menos cinco muestras, salto <1.5 m,
velocidad radial <=5 m/s; combina mínimos cuadrados 70% y mediana 30%, filtra con
alpha=0.2 y zona muerta 0.05. Pero su velocidad procedente del comando es
`2.11*linear.x-2.21`: no representa el throttle normalizado RC. Fusiona con pesos
0.2/0.85 y recorta negativos a cero. Copiar esta conversión impediría un TTC fiable
en el RC. Tampoco resuelve la reversa ni la caducidad.

`hotel_joy.launch.py` y `gz_spawn.launch.py` incluyen AEB y lidar_data. Los launch
Ackermann examinados conectan el mux directamente con controladores de simulación;
levantar además AEB allí no lo coloca necesariamente en serie. Se implementa una
cadena RC nueva, inequívoca y sin controladores de simulación.

## Decisiones RC

Se reutiliza la proyección geométrica y el umbral histórico 0.45 s. Se descartan
contracorriente, calibración de velocidad Hotel y la condición fija d_min<=1.4.
Neutralizar throttle es lo autorizado para este ESC; aplicar -2 sería incompatible.

Sin odometría o velocidad medida no se puede convertir throttle en m/s. El RC usa
un **límite superior físico configurable** de velocidad para calcular TTC, completo
ante cualquier demanda no nula (no multiplicado por throttle). Si ese límite
supera la velocidad real, resulta conservador para movimiento longitudinal frente
a obstáculos estáticos. Los valores iniciales 2 m/s adelante y 1 m/s atrás NO son
mediciones: `calibration_confirmed=false` impide movimiento hasta verificarlos.
No se ha añadido una odometría ficticia ni otro driver Ackermann.

Se conserva una envolvente circular de 0.45 m, pendiente de comprobar desde el
LiDAR hasta el extremo más lejano del carro. Se comprueba hemisferio delantero o
trasero según dirección; para coast se recuerdan ambas direcciones si cambia el
comando. Una colisión retiene la dirección de riesgo hasta despejarla. La ausencia
de cobertura trasera impide reversa. Esto no es predicción completa del barrido de
Ackermann ni de obstáculos móviles: el steering normalizado no es yaw-rate.

Para liberar se exige TTC>=0.65 s, al menos tres scans distintos y 0.3 s continuos
de despeje. El umbral separado evita chatter. Cualquier fallo interrumpe el conteo.
Retirar demanda de throttle no borra una colisión presente. Con el deadman mantenido
puede reanudar después del despeje: soltar throttle/deadman antes de retirar el
obstáculo en las pruebas.

NaN, infinito y fuera de rango son desconocidos, no prueba de camino libre. Se
tolera un número pequeño aislado; scan vacío, insuficiente o sector ciego extenso
ordena neutral inmediatamente. Así un scan de infinitos no autoriza movimiento.
En espacios abiertos esta política puede impedir circular: medir cobertura antes
de ajustar parámetros; no convertir fallos del sensor en distancias libres.

## Hardware preservado

Única interfaz: `ybeb_node.py`, entrada `/cmd_vel_stamped` TwistStamped.
Canal 1 throttle: `PWM=91+27*linear.x`; neutral 91; límites efectivos 80.2–101.8.
Canal 4 steering: `PWM=127.5+105*angular.z`; centro 127.5; límites 75–180.
Rosmaster convierte el valor a entero: neutral steering efectivo 127; extremos
throttle 80 y 101. No se cambió la calibración original.

El puerto LiDAR se conserva:
`/dev/serial/by-id/usb-Silicon_Labs_CP2102_USB_to_UART_Bridge_Controller_0001-if00-port0`.
`rplidar_ros/rplidar_composition`, 115200, `laser_frame`, inverted=false,
angle_compensate=true. Se usa QoS sensor BEST_EFFORT en AEB: recibe productores
BEST_EFFORT o RELIABLE; la prueba ROS usa un scan sintético BEST_EFFORT. No se ha
verificado un endpoint físico ni abierto ese puerto durante la implementación.

En el computador están joy 3.3.0, teleop_twist_joy 2.6.5 y twist_mux 4.4.0/Jazzy.
No está rplidar_ros en el entorno base probado. Rosmaster_Lib es una biblioteca del
fabricante, no un paquete ROS resoluble aquí. Ambos deben comprobarse en la Pi;
no se instalaron paquetes ni se abrió hardware.

## Comandos de auditoría y verificación

Ejecutados con rutas absolutas o desde los directorios indicados:

```bash
find /home/lenovo/mrad_ws_2602_hotel -name .git -print
git -C /home/lenovo/mrad_ws_2602_hotel/src log --all --date=short \
  --format='%h %ad %s' -- hotel_bringup/hotel_bringup/aeb_node.py \
  hotel_bringup/hotel_bringup/lidar_data.py
git -C /home/lenovo/mrad_ws_2602_hotel/src show 16cd27f:hotel_bringup/hotel_bringup/aeb_node.py
git -C /home/lenovo/mrad_ws_2602_hotel/src show 0557bc7 -- hotel_bringup/hotel_bringup/aeb_node.py
git -C /home/lenovo/mrad_ws_2602_hotel/src show 61d1d45 -- hotel_bringup/hotel_bringup/aeb_node.py
cat /home/lenovo/mrad_ws_2602_hotel/src/hotel_bringup/hotel_bringup/aeb_node.py
cat /home/lenovo/mrad_ws_2602_hotel/src/hotel_bringup/hotel_bringup/lidar_data.py
```

La lectura del historial desde la raíz exterior del workspace falló: su Git útil
está en `src`. Ese fallo no provocó checkout ni cambios de branch.

Compilación y tests en copia temporal, después aplicados sólo al paquete R:

```bash
source /opt/ros/jazzy/setup.bash
colcon build --packages-select ybeb_2602_zulu
source install/local_setup.bash
python3 -B -m pytest -q test/test_safety_core.py  # desde raíz del paquete
python3 -B -m pytest -q -s src/ybeb_2602_zulu/test/test_ros_pipeline.py
colcon test --packages-select ybeb_2602_zulu --event-handlers console_direct+
colcon test-result --verbose
```

Los cuatro `ros2 topic info` se ejecutan dentro del test con `--no-daemon
--spin-time 5`, dominio 187 y sólo localhost. El launch real del paquete arranca
con `start_lidar=false`, `start_hardware=false`; el test inyecta FakeRobot en la
clase hardware y nunca importa Rosmaster. El primer intento dentro del sandbox
falló por sockets DDS prohibidos; se repitió con permiso fuera del sandbox. La
espera inicial de descubrimiento se ajustó de 2 a 6 s después de observar que
Fast DDS tardaba aproximadamente 4 s en conectar sensor y suscripciones.
