# Verificación RC migrada a Hotel — 2026-09-26

Fuente: `/home/lenovo/mrad_ws_2602_hotel/src`. Entorno local ROS 2 Jazzy,
Python 3.12.3, pytest 7.4.4. No se abrió Rosmaster ni se movió el vehículo.

## Resultado

- `colcon build` completo desde Hotel: **PASS, 12 paquetes**.
- Suite RC migrada (`hotel_bringup/test/test_rc_*.py`): **63 PASS, 0 FAIL**.
- Ejecución completa de los cuatro paquetes: **79 PASS, 5 FAIL, 4 SKIP**, 0 errores.
- Linters del nuevo paquete hardware: **2 PASS**; copyright heredado omitido.
- Las 58 pruebas funcionales de la referencia anterior se conservan, junto a
  sus dos linters de hardware; se añaden cinco comprobaciones de migración.

La referencia del respaldo tenía 60 PASS/0 FAIL/1 SKIP. Ese número no se presenta
como resultado nuevo de todo Hotel. Los cinco fallos actuales son **preexistentes**:
antes de editar se ejecutaron y fallaron los mismos tests globales de estilo:

| Paquete | PASS | FAIL | SKIP | Fallos |
|---|---:|---:|---:|---|
| hotel_bringup | 63 | 2 | 1 | test_flake8, test_pep257 |
| hotel_wall_following | 1 | 1 | 1 | test_flake8 |
| hotel_ttc_follow_the_gap | 13 | 2 | 1 | test_flake8, test_pep257 |
| ybeb_2602_zulu | 2 | 0 | 1 | Ninguno |

Línea base antes de editar: flake8 reportaba 530 incidencias en bringup, 5 en Wall
y 248 en Gap; pep257 fallaba en bringup y Gap. No se deshabilitaron tests ni se
reformateó el código histórico para esconder esa deuda. El código RC nuevo/migrado
tiene una comprobación separada flake8/pep257 que sí pasa.

## Cobertura conservada

| Caso | Resultado |
|---|---|
| Pass-through sin riesgo | PASS |
| TTC bajo y obstáculo frontal con demanda positiva | PASS |
| Obstáculo lateral sin aproximación longitudinal | PASS |
| Inf/NaN aislados y scans totalmente inválidos | PASS |
| Velocidad cero sin dividir por cero | PASS |
| Pérdida de scan y de comando mux | PASS |
| Pérdida de AEB → watchdog ybeb independiente | PASS con FakeRobot |
| Histéresis, hold y scans nuevos | PASS |
| Saturación de steering/throttle | PASS |
| NaN/inf en comandos nunca produce PWM inválido | PASS |
| Reversa, coast, cobertura incompleta, sector ciego y metadatos | PASS |
| Sellos antiguos, futuros, vacíos, scan repetido/frame incorrecto | PASS |
| Prioridades joy > gap > wall > nav y cesión por timeout | PASS |
| Teleop instalado: YAML RC, escalas y deadman | PASS |
| Imports/launch/config/dependencias instalados | PASS |
| Wall/Gap, límites y silencio al desactivar/perder heartbeat | PASS |
| YAML independientes de Wall/Gap mediante parámetros ROS efectivos | PASS |

Los cinco casos adicionales verifican separación de launch PC/Pi, defaults de
calidad compartidos, ausencia del salto ±1.4 en ambos signos del controlador
histórico y estilo del código RC.

El test de integración inicia el launch **PC** real con Wall/Gap opcionales
presentes pero inactivos, usa twist_mux y teleop instalados y construye la interfaz
ybeb con FakeRobot en memoria. Nunca se importa Rosmaster_Lib en ese camino.
No se ejecuta el launch Pi con actuadores ni el driver RPLIDAR real.

El launch PC utiliza argumentos separados `wall_config` y `gap_config`. El test
carga YAML distintos con `steering_gain` de 0.31 y 0.42, y consulta los parámetros
de `/rc_wall` y `/rc_gap` para comprobar que cada nodo recibe su configuración.
Esos valores son exclusivos de la prueba; los YAML operativos conservan 0.7.

`/cmd_vel_joy`, `/cmd_vel_mux` y `/cmd_vel_stamped` se comprueban como
`geometry_msgs/msg/TwistStamped`; `/scan` como `sensor_msgs/msg/LaserScan`.
Se comprueba el único emisor de `/cmd_vel_mux` y `/cmd_vel_stamped`, y la suscripción
BEST_EFFORT de AEB. Los comandos `ros2 topic info` se ejecutan dentro del test con
`--no-daemon --spin-time 5`. Los contadores incluyen sondas y fuentes de prueba.

## Comandos y evidencia

Desde `/home/lenovo/mrad_ws_2602_hotel`, entorno base Jazzy sin overlays de respaldo:

```bash
source /opt/ros/jazzy/setup.bash
colcon build
source install/local_setup.bash
colcon test --packages-select hotel_bringup hotel_wall_following \
  hotel_ttc_follow_the_gap ybeb_2602_zulu \
  --return-code-on-test-failure --event-handlers console_direct+
colcon test-result --verbose
```

El comando de tests devolvió 1 por los cinco tests de estilo identificados.
Se utilizó `PYTHONDONTWRITEBYTECODE=1`, de ahí los avisos de bytecode desactivado.
También hubo warnings fork/multithreading de lint, sin fallo funcional RC.

XML verificables en este workspace:

```text
build/hotel_bringup/pytest.xml
build/hotel_wall_following/pytest.xml
build/hotel_ttc_follow_the_gap/pytest.xml
build/ybeb_2602_zulu/pytest.xml
```

Registro de esta sesión local, fuera del repositorio:

```text
/tmp/hotel_rc_migration_j8sgb03b/build.log
/tmp/hotel_rc_migration_j8sgb03b/tests.log
/tmp/hotel_rc_migration_j8sgb03b/hotel_bringup_baseline_lint.log
/tmp/hotel_rc_migration_j8sgb03b/hotel_wall_following_baseline_lint.log
/tmp/hotel_rc_migration_j8sgb03b/hotel_ttc_follow_the_gap_baseline_lint.log
```

Los tests ROS se autorizaron fuera del sandbox para disponer de sockets DDS, pero
fueron localhost con dominios 187/188. El dominio operativo propuesto 27 no se usó.
No se ejecutó SSH/SCP, no se abrieron actuadores y no se publicó al vehículo.

## Pendientes

No se acredita despliegue en Pi, enlace Wi-Fi físico, RPLIDAR real, F710, ESC,
servo, velocidad medida, coast ni distancia de parada. Mantener el bloqueo de
calibración y seguir el [README](../README.md) para aceptar el vehículo. Wall y
Gap permanecen aproximadamente al 25%; navegación completa queda fuera de fase.
