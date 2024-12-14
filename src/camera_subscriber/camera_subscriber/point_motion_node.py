#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Point, Twist
from rclpy.duration import Duration

class MotionController(Node):
    def __init__(self):
        super().__init__('motion_controller')
        self.cmd_vel_publisher = self.create_publisher(Twist, '/cmd_vel', 10)
        self.point_subscriber = self.create_subscription(Point, '/point', self.point_callback, 10)

        # Parametry rozmiaru okna
        self.declare_parameter('window_height', 240)
        self.declare_parameter('window_width', 320)
        self.window_height = self.get_parameter('window_height').get_parameter_value().integer_value
        self.window_width = self.get_parameter('window_width').get_parameter_value().integer_value

        self.get_logger().info(f'Uruchomiono MotionController z rozmiarem okna: {self.window_width}x{self.window_height}')
        
        # Parametry prędkości
        self.forward_speed = 0.1  # Prędkość liniowa (m/s)
        self.turn_speed = 0.1     # Prędkość obrotowa (rad/s)

        # Inicjalizacja czasu ostatniej wiadomości Point
        self.last_point_time = self.get_clock().now()
        self.timeout_duration = Duration(seconds=1.0)  # Czas po którym robot się zatrzyma, jeśli nie wykryto markera

        # Timer do sprawdzania, czy marker został wykryty w ostatnim czasie
        self.timer = self.create_timer(0.1, self.check_for_timeout)

    def point_callback(self, point_msg):
        try:
            # Aktualizacja czasu ostatniej otrzymanej wiadomości
            self.last_point_time = self.get_clock().now()

            point_x = point_msg.x
            point_y = point_msg.y
            middle_x = self.window_width / 2
            middle_y = self.window_height / 2
            twist_msg = Twist()

            # Kontrola przód/tył
            if point_y < middle_y:
                twist_msg.linear.x = self.forward_speed
                self.get_logger().info(f'Punkt powyżej środka Y: Jazda do przodu (y={point_y}, środek={middle_y})')
            else:
                twist_msg.linear.x = -self.forward_speed
                self.get_logger().info(f'Punkt poniżej środka Y: Jazda do tyłu (y={point_y}, środek={middle_y})')

            # Kontrola lewo/prawo
            if point_x < middle_x:
                twist_msg.angular.z = self.turn_speed
                self.get_logger().info(f'Punkt na lewo od środka X: Skręt w lewo (x={point_x}, środek={middle_x})')
            else:
                twist_msg.angular.z = -self.turn_speed
                self.get_logger().info(f'Punkt na prawo od środka X: Skręt w prawo (x={point_x}, środek={middle_x})')

            self.cmd_vel_publisher.publish(twist_msg)
        except Exception as e:
            self.get_logger().error(f"Błąd podczas przetwarzania wiadomości Point: {e}")

    def check_for_timeout(self):
        # Sprawdzenie, czy minął czas od ostatniego wykrycia markera
        now = self.get_clock().now()
        time_since_last_point = now - self.last_point_time

        if time_since_last_point > self.timeout_duration:
            # Jeśli marker nie został wykryty w określonym czasie, zatrzymujemy robota
            twist_msg = Twist()
            twist_msg.linear.x = 0.0
            twist_msg.angular.z = 0.0
            self.cmd_vel_publisher.publish(twist_msg)
            self.get_logger().info('Marker nie jest wykrywany. Zatrzymanie robota.')

def main(args=None):
    rclpy.init(args=args)
    node = MotionController()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        # Zatrzymanie robota przed zamknięciem
        twist_msg = Twist()
        twist_msg.linear.x = 0.0
        twist_msg.angular.z = 0.0
        node.cmd_vel_publisher.publish(twist_msg)
        
        node.get_logger().info('Zamykanie MotionController.')
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
