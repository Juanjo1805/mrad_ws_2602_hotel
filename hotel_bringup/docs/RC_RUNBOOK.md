# RC: operación desde Hotel y despliegue PC/Raspberry

La fuente de verdad es `/home/lenovo/mrad_ws_2602_hotel`, con Git en `src`.
El respaldo de `proyecto_rc` no se modifica ni se utiliza como workspace principal.
La descripción previa que situaba mux/AEB en Raspberry queda sustituida por ésta.

## Responsabilidades

- **PC:** joy/teleop, twist_mux, AEB, Wall y Gap opcionales, navegación futura.
- **Raspberry:** RPLIDAR si está conectado allí; ybeb/Rosmaster/watchdog/actuadores.
- **DDS/Wi-Fi:** `/scan` de Pi a PC y `/cmd_vel_stamped` de PC a Pi.
- No hay bridges ni un segundo driver Ackermann.

## Compilar y probar en PC

En una terminal limpia:

```bash
cd /home/lenovo/mrad_ws_2602_hotel
source /opt/ros/jazzy/setup.bash
colcon build
source install/local_setup.bash
colcon test --packages-select hotel_bringup hotel_wall_following \
  hotel_ttc_follow_the_gap ybeb_2602_zulu \
  --return-code-on-test-failure --event-handlers console_direct+
colcon test-result --verbose
```

Los tests RC usan FakeRobot y dominios localhost 187/188. El build no abre serial.
La ejecución completa aún muestra cinco fallos de lint preexistentes: ver
[TEST_REPORT.md](TEST_REPORT.md). La aceptación RC específica tiene 63 PASS:

```bash
python3 -B -m pytest -q src/hotel_bringup/test/test_rc_*.py
```

## Despliegue y primera puesta en servicio

El [README principal](../README.md#procedimiento-de-puesta-en-servicio) contiene
el procedimiento completo, los comandos SSH/SCP y las tablas de calibración.
Copiar sólo `src/ybeb_2602_zulu` desde Hotel a la Pi; guardar fuentes anteriores
fuera de `src`, sin borrarlas, para evitar paquetes duplicados. Compilar de nuevo
en la arquitectura de la Pi; no copiar `build/`/`install/` del PC. La ruta
`~/ros2_ws_2602` usada en los ejemplos es una suposición que el operador debe verificar.

En ambos equipos para la sesión real:

```bash
source /opt/ros/jazzy/setup.bash
export ROS_DOMAIN_ID=27
unset ROS_LOCALHOST_ONLY ROS_STATIC_PEERS
export ROS_AUTOMATIC_DISCOVERY_RANGE=SUBNET
```

Después cargar en cada equipo su instalación correspondiente. LiDAR sin actuadores
en **Pi**:

```bash
ros2 launch ybeb_2602_zulu rc_pi_bringup.launch.py \
  start_lidar:=true start_hardware:=false
```

Mux/AEB todavía bloqueados por calibración, en **PC**:

```bash
ros2 launch hotel_bringup rc_pc_bringup.launch.py calibration_confirmed:=false
ros2 launch hotel_bringup joystick_rc.launch.py \
  linear_scale:=0.1 steering_scale:=0.25
```

Son procesos para terminales separadas. Verificar `/scan` en PC, `/joy`,
`/cmd_vel_joy`, `/cmd_vel_mux`, `/cmd_vel_stamped`, los tipos y `/aeb/reason`.
No cambiar el interlock para ocultar un fallo; `calibration_required` tiene
precedencia y su TTC infinito no prueba scan válido.

## Hardware: sólo Raspberry y bajo supervisión

Con carro sujeto, ruedas levantadas, tracción inicialmente sin potencia y corte
físico accesible, iniciar ybeb por separado permite probar su watchdog:

```bash
ros2 run ybeb_2602_zulu ybeb_node --ros-args \
  --params-file "$HOME/ros2_ws_2602/src/ybeb_2602_zulu/config/hardware_rc.yaml"
```

Comprobar neutral 91, centro 127.5 (entero 127), signos, deadman y parámetros físicos.
Después de calibrar, el PC puede relanzar AEB con `calibration_confirmed=true`;
no iniciar un segundo hardware. Comprobar frenado ante blanco blando y neutral
ante pérdida de joystick, LiDAR, AEB y red. Para probar watchdog, dejar vivo ybeb
en Pi y terminar el bringup del PC. Medir actuación real, no sólo mensajes.

## Operación integrada posterior

Cerrar procesos individuales antes de usar estos launch para evitar duplicados.
Pi:

```bash
ros2 launch ybeb_2602_zulu rc_pi_bringup.launch.py \
  start_lidar:=true start_hardware:=true
```

PC, sólo tras calibración y aceptación:

```bash
ros2 launch hotel_bringup rc_pc_bringup.launch.py \
  start_joystick:=true start_wall:=false start_gap:=false \
  calibration_confirmed:=true
```

La primera prueba en suelo es recta y a velocidad mínima, sin Wall/Gap/Nav. Registrar
coast y distancia hasta inmovilidad. Los valores 2 m/s, 1 m/s, radio 0.45 m y yaw 0°
son provisionales. La aceptación física sigue pendiente; no se realizaron estas
operaciones sobre el vehículo durante la migración.
