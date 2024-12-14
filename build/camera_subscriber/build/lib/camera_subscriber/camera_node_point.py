#!/usr/bin/env python3
import rclpy  # Python Client Library for ROS 2
from rclpy.node import Node  # Handles the creation of nodes
from sensor_msgs.msg import Image  # Image is the message type
from geometry_msgs.msg import Point  # Import Point message type
from cv_bridge import CvBridge  # ROS2 package to convert between ROS and OpenCV Images
import cv2  # Python OpenCV library
import numpy as np


class MinimalSubscriber(Node):
    def __init__(self):
        super().__init__('minimal_subscriber')
        self.window_name = "camera"
        # Subskrybent obrazu (dla testów z kamerą)
        self.subscription = self.create_subscription(
            Image, 'image_raw', self.listener_callback, 10)
        self.subscription  # prevent unused variable warning

        # Publisher dla wiadomości typu Point
        self.publisher = self.create_publisher(Point, '/point', 10)

        self.point = None  # Przechowuje współrzędne kliknięcia

    def listener_callback(self, image_data):
        # Zamiast obrazu z kamery używamy pustej ramki
        cv_image = np.zeros((512, 700, 3), np.uint8)
        if self.point is not None:
            # Rysowanie prostokąta wokół klikniętego punktu
            cv2.rectangle(cv_image, self.point, (self.point[0] + 200, self.point[1] + 200), (0, 255, 0), 3)

        # Wyświetlanie okna z prostokątem
        cv2.imshow(self.window_name, cv_image)
        cv2.waitKey(25)
        cv2.setMouseCallback(self.window_name, self.draw_rectangle)

    def draw_rectangle(self, event, x, y, flags, param):
        if event == cv2.EVENT_LBUTTONDOWN:  # Jeśli kliknięto lewy przycisk myszy
            self.point = (x, y)  # Zapisz współrzędne kliknięcia
            self.publish_point(x, y)  # Publikuj punkt

    def publish_point(self, x, y):
        # Utwórz wiadomość Point
        point_msg = Point()
        point_msg.x = float(x)
        point_msg.y = float(y)
        point_msg.z = 0.0  # Opcjonalnie, można ustawić wartość z na stałą
        # Publikuj wiadomość
        self.publisher.publish(point_msg)
        self.get_logger().info(f'Published point: ({x}, {y}, {point_msg.z})')


def main(args=None):
    rclpy.init(args=args)
    minimal_subscriber = MinimalSubscriber()
    try:
        rclpy.spin(minimal_subscriber)
    except KeyboardInterrupt:
        pass
    finally:
        # Zakończ działanie węzła i zamknij okno
        minimal_subscriber.destroy_node()
        rclpy.shutdown()
        cv2.destroyAllWindows()


if __name__ == '__main__':
    main()
