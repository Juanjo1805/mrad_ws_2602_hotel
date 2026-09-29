# RC — Guía rápida de prueba

## Arquitectura actual

```text
PC: Joystick → /joy → teleop_twist_joy → /cmd_vel_joy → twist_mux → /cmd_vel_mux
                                                         │ Wi-Fi / DDS
Pi: /cmd_vel_mux + /scan (RPLIDAR) → AEB → /cmd_vel_stamped → ybeb_node → Rosmaster → carro
```

## Qué corre en el PC

Ejecutables: `joy joy_node`, `teleop_twist_joy teleop_node`, `twist_mux twist_mux`. Launch: `rc_pc_bringup.launch.py`.

```bash
cd /home/lenovo/mrad_ws_2602_hotel
source /opt/ros/jazzy/setup.bash
source install/local_setup.bash
export ROS_DOMAIN_ID=27
export ROS_AUTOMATIC_DISCOVERY_RANGE=SUBNET
ros2 launch hotel_bringup rc_pc_bringup.launch.py
```

Observar: `/joy` → `/cmd_vel_joy` → `/cmd_vel_mux`.

## Qué debo ver en el PC

| Tópico | Qué comprobar |
|---|---|
| `/joy` | Cambian ejes y botones. |
| `/cmd_vel_joy` | Throttle llega hasta ±0.4; steering hasta ±0.5. |
| `/cmd_vel_mux` | Es idéntico a `/cmd_vel_joy`. |

`/joy` cambia y `/cmd_vel_joy` no: revisar teleop, deadman, ejes y YAML. `/cmd_vel_joy` bien y `/cmd_vel_mux` no: revisar tópico y timeout de `twist_mux`.

## Qué corre en la Raspberry

Propios: `ybeb_2602_zulu aeb_node`, `ybeb_2602_zulu ybeb_node`. Externo: RPLIDAR con `rplidar_ros`. Launch: `rc_pi_bringup.launch.py`.

```bash
cd ~/ros2_ws_2602
source /opt/ros/jazzy/setup.bash
source install/local_setup.bash
export ROS_DOMAIN_ID=27
export ROS_AUTOMATIC_DISCOVERY_RANGE=SUBNET
```

### Sólo LiDAR

`ros2 launch ybeb_2602_zulu rc_pi_bringup.launch.py start_lidar:=true start_aeb:=false start_hardware:=false`

### LiDAR + AEB

`ros2 launch ybeb_2602_zulu rc_pi_bringup.launch.py start_lidar:=true start_aeb:=true start_hardware:=false`

### Sistema completo

`ros2 launch ybeb_2602_zulu rc_pi_bringup.launch.py start_lidar:=true start_aeb:=true start_hardware:=true`

Cerrar el launch anterior antes de iniciar otra etapa.

## Qué debo ver en la Pi

| Tópico | Qué comprobar |
|---|---|
| `/cmd_vel_mux` | Llega del PC por Wi-Fi. |
| `/scan` | Publica el RPLIDAR. |
| `/cmd_vel_stamped` | Con camino libre, es igual a `/cmd_vel_mux`. |

Con obstáculo peligroso: `twist.linear.x != 0` en `/cmd_vel_mux` → `twist.linear.x = 0` en `/cmd_vel_stamped` por el AEB.

## Calibración del carro

Throttle: `−0.4 … +0.4`. Steering: `−0.5 … +0.5`. Servo: mínimo `74°`, centro `103°`, máximo `129°`.

**103° es el centro calibrado de ESTE vehículo. No usar 90°.** Confirmar físicamente la orientación izquierda/derecha.

## Depuración rápida

### El carro no se mueve

Revisar en orden: `/joy` → `/cmd_vel_joy` → `/cmd_vel_mux` → `/cmd_vel_mux` en Pi → `/cmd_vel_stamped` → `ybeb_node` → Rosmaster.

### `/cmd_vel_joy` sólo llega a valores pequeños

Revisar `joystick_rc.yaml`: throttle `0.4`, steering `0.5`; nunca `0.1 / 0.25`.

### La Pi no ve `/cmd_vel_mux`

Revisar mismo `ROS_DOMAIN_ID`, misma Wi-Fi, `ROS_AUTOMATIC_DISCOVERY_RANGE=SUBNET` y descubrimiento DDS.

### No aparece `/scan`

Revisar RPLIDAR conectado, `rplidar_ros`, puerto, permisos y launch LiDAR.

### `/cmd_vel_mux` tiene valor pero `/cmd_vel_stamped` queda en cero

Revisar `/scan`, AEB, obstáculo, timeout y TTC.

### `/cmd_vel_stamped` está bien pero el carro no se mueve

Revisar `ybeb_node`, Rosmaster, puerto serie, neutral throttle y alimentación.

## Comandos de echo

PC:

```bash
ros2 topic echo /joy
ros2 topic echo /cmd_vel_joy
ros2 topic echo /cmd_vel_mux
```

Pi:

```bash
ros2 topic echo /cmd_vel_mux
ros2 topic echo /scan --once
ros2 topic hz /scan
ros2 topic echo /cmd_vel_stamped
```

## Objetivo actual

- [ ] Joystick mueve el carro.
- [ ] LiDAR publica `/scan` correctamente.
- [ ] AEB detiene el carro ante un obstáculo.

Follow the Wall y Follow the Gap están pausados hasta completar estas tres pruebas.
