# mrad_ws_2602_hotel

Workspace ROS 2 para simulacion, descripcion, control y navegacion reactiva del robot del proyecto Hotel. Incluye paquetes para levantar el robot en Gazebo, publicar sensores, controlar velocidades, seguir lineas/carriles, seguir paredes y navegar con Follow the Gap usando LiDAR.

## Paquetes principales

### hotel_bringup

Paquete encargado de lanzar el sistema completo del robot. Contiene launch files para Gazebo, robot_state_publisher, joystick, teleoperacion, `twist_mux`, nodos auxiliares de LiDAR y frenado de emergencia. El launch principal de simulacion es `gz_spawn.launch.py`, que carga el robot, abre los bridges de Gazebo a ROS 2, inicia controladores y conecta las fuentes de velocidad.

Topicos importantes:

- `/scan`: lectura del LiDAR.
- `/joy`: datos del joystick.
- `/cmd_vel_joy`: comando de velocidad desde joystick.
- `/cmd_vel_key`: comando de velocidad desde teclado.
- `/cmd_vel_ctrl`: comando de control autonomo.
- `/cmd_vel_lka`: comando del seguidor de linea.
- `/cmd_vel_mux`: salida combinada del `twist_mux`.
- `/cmd_vel_out`: salida filtrada por el nodo de seguridad.
- `/diffdrive_controller/cmd_vel`: entrada final del controlador diferencial.
- `/lidar/vctrl`: velocidad usada por controladores reactivos.
- `/lidar/d_min`: distancia minima detectada por LiDAR.
- `/lidar/front_scan`: sector frontal del LiDAR.
- `/dist_min`: informacion frontal usada por controladores de seguridad y pared.

### hotel_control

Paquete base de control construido con `ament_cmake`. Actualmente no define nodos propios, pero sirve como paquete de soporte para integrar configuraciones, controladores o codigo C++ relacionado con control del robot.

Topicos importantes:

- No publica ni se suscribe a topicos directamente en el estado actual.
- El control efectivo del robot se conecta principalmente desde `hotel_bringup`, `twist_mux` y los controladores de ROS 2.

### hotel_description

Paquete que contiene la descripcion del robot en archivos Xacro/URDF. Define la estructura fisica del robot, sensores, links, joints, propiedades, inerciales y configuraciones para modelos diferenciales y Ackermann. Tambien incluye sensores como LiDAR, IMU, camaras frontales/laterales y camara de profundidad.

Topicos importantes generados por sensores:

- `/scan`: LiDAR.
- `/imu` o `/imu/data`: IMU segun el bridge usado.
- `/camera/image`: camara frontal.
- `/camera/camera_info`: informacion de la camara frontal.
- `/left_camera/image`: camara izquierda para carril.
- `/left_camera/camera_info`: informacion de camara izquierda.
- `/right_camera/image`: camara derecha para carril.
- `/right_camera/camera_info`: informacion de camara derecha.
- `/depth_camera/points`: nube de puntos de camara de profundidad.

### hotel_gazebo

Paquete con mundos, mapas, archivos RViz y configuraciones de bridge entre Gazebo y ROS 2. Permite conectar sensores simulados de Gazebo con topicos ROS 2 usando archivos como `topic_bridge.yaml` y `topic_bridge_ackermann.yaml`.

Topicos importantes del bridge:

- `/clock`: reloj de simulacion.
- `/scan`: LiDAR desde Gazebo hacia ROS 2.
- `/camera/image` y `/camera/camera_info`: camara frontal.
- `/left_camera/image` y `/left_camera/camera_info`: camara izquierda.
- `/right_camera/image` y `/right_camera/camera_info`: camara derecha.
- `/depth_camera/points`: nube de puntos.
- `/imu` o `/imu/data`: IMU.

### hotel_line_extractor

Paquete para extraer lineas o seguir carril usando sensores. Incluye un nodo de extraccion de segmentos desde LiDAR (`line_extractor_node`) y un nodo de asistencia de mantenimiento de carril por camara (`lka_node`). El launch `lka.launch.py` ejecuta el nodo de seguimiento de carril, usando por defecto la imagen de la camara izquierda.

Topicos importantes:

- `/scan`: entrada LiDAR para extraccion de segmentos.
- `/line_markers`: marcadores visuales de lineas detectadas.
- `/line_segments`: segmentos de linea publicados como poses.
- `/left_camera/image`: imagen usada por defecto para seguimiento de carril.
- `/joy`: habilitacion por joystick, si esta activa.
- `/cmd_vel_lka`: comando de velocidad generado por el seguidor de carril.
- `/lka/lateral_error`: error lateral detectado.
- `/lka/heading_error`: error de orientacion.
- `/lka/debug_image`: imagen de depuracion del procesamiento.

### hotel_ttc_follow_the_gap

Paquete de navegacion reactiva basado en LiDAR. Combina Follow the Gap con TTC, que significa Time To Collision. El nodo `ttc_gap_finder` analiza el LiDAR, calcula zonas seguras, crea burbujas alrededor de obstaculos cercanos y selecciona el mejor angulo libre. El nodo `ttc_control` convierte ese angulo en comandos de velocidad y giro, usando ademas un boton de seguridad del joystick.

Topicos importantes:

- `/scan`: entrada del LiDAR.
- `/lidar/vctrl`: velocidad usada para calcular TTC y burbujas.
- `/gap_angle`: angulo elegido hacia el mejor espacio libre.
- `/min_ttc`: menor tiempo estimado antes de colision.
- `/joy`: habilitacion por boton RB.
- `/cmd_vel_ctrl`: comando de velocidad generado por el controlador.

### hotel_wall_following

Paquete para seguimiento de pared usando LiDAR y control PD. El nodo `dist_finder` calcula el error entre la distancia deseada y la distancia proyectada a la pared. El nodo `wall_follower_control` usa ese error para calcular giro y velocidad, reduciendo la velocidad cuando el error aumenta y aplicando una correccion extra si detecta obstaculos frontales.

Topicos importantes:

- `/scan`: entrada del LiDAR.
- `/error`: error lateral respecto a la pared.
- `/diagiz_dist`: distancia diagonal auxiliar.
- `/dist_min`: informacion frontal usada para evitar obstaculos.
- `/joy`: habilitacion por boton RB.
- `/cmd_vel_ctrl`: comando de velocidad generado por el seguidor de pared.

## Comandos de ejecucion

Antes de lanzar, compilar y cargar el entorno:

```bash
colcon build
source install/setup.bash
```

Lanzar simulacion principal en Gazebo:

```bash
ros2 launch hotel_bringup gz_spawn.launch.py
```

Lanzar seguimiento de linea/carril:

```bash
ros2 launch hotel_line_extractor lka.launch.py
```

Lanzar Follow the Gap con TTC:

```bash
ros2 launch hotel_ttc_follow_the_gap gap_follow.launch.py
```

Lanzar seguimiento de pared:

```bash
ros2 launch hotel_wall_following wall_following.launch.py
```
