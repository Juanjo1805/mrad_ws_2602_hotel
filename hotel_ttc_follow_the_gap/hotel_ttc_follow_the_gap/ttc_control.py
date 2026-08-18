#!/usr/bin/env python3
from geometry_msgs.msg import TwistStamped
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Joy
from std_msgs.msg import Float32


PARAMETERS = (
    # TTC de referencia para reducir velocidad cuando hay riesgo cercano.
    ('ttc_min', 0.6),
    # Velocidad maxima deseada cuando el camino esta libre, en m/s.
    ('v_max', 1.6),
    # Velocidad minima permitida cuando el robot esta habilitado, en m/s.
    ('v_min', 0.15),
    # Ganancia proporcional que convierte el angulo del gap en giro.
    ('kp_steering', 1.2),
    # Limite maximo del comando de giro, en rad/s.
    ('max_steering', 2.5),
    # Reduccion de velocidad al girar; mas alto = mas lento en curvas.
    ('steering_slowdown', 1.5),
    # Zona muerta del angulo; ignora correcciones pequenas para evitar ruido.
    ('angle_deadband', 0.05),
)


class TTCControl(Node):
    def __init__(self):
        super().__init__('ttc_control')

        self._load_parameters()
        self._reset_state()
        self._create_ros_interfaces()

        self.get_logger().info(
            f'TTCControl OK | v_max={self.v_max} v_min={self.v_min} '
            f'kp={self.kp} ttc_ref={self.ttc_ref}'
        )

    def _load_parameters(self):
        for name, default in PARAMETERS:
            self.declare_parameter(name, default)

        self.ttc_ref = self.get_parameter('ttc_min').value
        self.v_max = self.get_parameter('v_max').value
        self.v_min = self.get_parameter('v_min').value
        self.kp = self.get_parameter('kp_steering').value
        self.max_steering = self.get_parameter('max_steering').value
        self.slow_gain = self.get_parameter('steering_slowdown').value
        self.deadband = self.get_parameter('angle_deadband').value

    def _reset_state(self):
        self.rb_pressed = False
        self.gap_angle = 0.0
        self.min_ttc = float('inf')

    def _create_ros_interfaces(self):
        self.create_subscription(Float32, '/gap_angle', self.gap_callback, 10)
        self.create_subscription(Float32, '/min_ttc', self.ttc_callback, 10)
        self.joy_sub = self.create_subscription(
            Joy, '/joy', self.joy_callback, 10
        )
        self.cmd_pub = self.create_publisher(TwistStamped, '/cmd_vel_ctrl', 10)
        self.create_timer(0.02, self.compute_and_publish)

    def gap_callback(self, msg: Float32):
        self.gap_angle = msg.data

    def ttc_callback(self, msg: Float32):
        self.min_ttc = msg.data

    def joy_callback(self, msg: Joy):
        self.rb_pressed = (msg.buttons[5] == 1)

    def compute_and_publish(self):
        steering = self._calculate_steering()
        velocity = self._calculate_velocity(steering)
        self._publish_command(velocity, steering)

    def _calculate_steering(self):
        angle = self.gap_angle if abs(self.gap_angle) >= self.deadband else 0.0
        steering = self.kp * angle
        steering = steering / (1.0 + abs(steering)) * self.max_steering
        steering = self._clamp(steering, -self.max_steering, self.max_steering)

        if steering > 0.1 and steering < 1.4:
            return 1.4
        if steering < -0.1 and steering > -1.4:
            return -1.4
        return steering

    def _calculate_velocity(self, steering):
        if self.min_ttc <= 0.01:
            scale = 0.0
        else:
            scale = min(self.min_ttc / self.ttc_ref, 1.0)

        base_vel = self.v_max * scale
        velocity = base_vel / (1.0 + self.slow_gain * abs(steering))

        # Preserve the original saturation order.
        velocity = max(self.v_max, velocity)
        return min(self.v_min, velocity)

    def _publish_command(self, velocity, steering):
        if not self.rb_pressed:
            velocity = 0.0
            steering = 0.0
            self.get_logger().info('Dead man activado')

        cmd = TwistStamped()
        cmd.header.stamp = self.get_clock().now().to_msg()
        cmd.header.frame_id = 'base_link'
        cmd.twist.linear.x = float(velocity)
        cmd.twist.angular.z = float(steering)
        self.cmd_pub.publish(cmd)

    @staticmethod
    def _clamp(value, lower, upper):
        return max(lower, min(upper, value))


def main(args=None):
    rclpy.init(args=args)
    rclpy.spin(TTCControl())
    rclpy.shutdown()


if __name__ == '__main__':
    main()
