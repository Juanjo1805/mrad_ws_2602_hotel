# RC Raspberry hardware package

Fuente de verdad: `mrad_ws_2602_hotel/src/ybeb_2602_zulu`.

Este paquete se compila también en el PC sin abrir hardware. Sólo se despliega
y ejecuta `ybeb_node` en la Raspberry. El LiDAR puede ejecutarse allí con
`rc_pi_bringup.launch.py`. No contiene ejecutables AEB, mux ni joystick.

La única interfaz consume `/cmd_vel_stamped` TwistStamped, con throttle
normalizado ±0.4 y steering ±0.5. Watchdog local 0.25 s, timer 20 Hz, neutral
91 y centro 127.5 (la biblioteca trunca a entero). Fórmulas originales conservadas.

`Rosmaster_Lib` se importa sólo al construir hardware real; build y tests usan
imports puros o inyección de FakeRobot. `safety_core.py`, `scan_support.py` y
`ros_support.py` son utilidades compartidas sin apertura de dispositivos al importar.

El PC ejecuta mux/AEB y recibe `/scan` por DDS/Wi-Fi; la Pi recibe el comando
final y lo valida de nuevo. El watchdog local no depende del enlace ni del AEB.

El launch Pi no abre dispositivos por defecto: `start_lidar=false`,
`start_hardware=false`. No ejecutar en el PC con hardware habilitado.

Procedimiento de despliegue, calibración y aceptación física:
[README principal](../hotel_bringup/README.md). Esta referencia está disponible
en el repositorio Hotel completo; consultar ese README antes de desplegar.
