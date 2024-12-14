#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from geometry_msgs.msg import Point
from cv_bridge import CvBridge
import cv2
import numpy as np
import cv2.aruco as aruco

class ArucoDetector(Node):
    def __init__(self):
        super().__init__('aruco_detector')
        self.window_name = "camera"
        
        self.subscription = self.create_subscription(Image, '/camera/image_raw', self.listener_callback, 10)
        self.publisher = self.create_publisher(Point, '/point', 10)
        self.bridge = CvBridge()

        # Ustawienie słownika i parametrów ArUco
        self.aruco_dict = aruco.getPredefinedDictionary(aruco.DICT_6X6_250)
        self.parameters = aruco.DetectorParameters()  # Zmieniona linia

    def listener_callback(self, image_data):
        try:
            cv_image = self.bridge.imgmsg_to_cv2(image_data, desired_encoding='bgr8')
        except Exception as e:
            self.get_logger().error(f"Nie udało się przekonwertować obrazu: {e}")
            return
        
        # Wykrywanie markerów ArUco
        gray = cv2.cvtColor(cv_image, cv2.COLOR_BGR2GRAY)
        corners, ids, _ = aruco.detectMarkers(gray, self.aruco_dict, parameters=self.parameters)
        
        if ids is not None:
            for i in range(len(ids)):
                # Rysowanie wykrytych markerów
                aruco.drawDetectedMarkers(cv_image, corners)
                
                # Obliczanie środka markera
                c = corners[i][0]
                center_x = int(np.mean(c[:, 0]))
                center_y = int(np.mean(c[:, 1]))
                
                # Rysowanie punktu na środku
                cv2.circle(cv_image, (center_x, center_y), 5, (0, 0, 255), -1)
                
                # Publikowanie punktu środka
                self.publisher.publish(Point(x=float(center_x), y=float(center_y), z=0.0))
                self.get_logger().info(f"Wykryto marker ArUco ID {ids[i][0]} na pozycji ({center_x}, {center_y})")
        else:
            self.get_logger().info("Nie wykryto żadnych markerów ArUco.")
        
        cv2.imshow(self.window_name, cv_image)
        cv2.waitKey(1)

def main(args=None):
    rclpy.init(args=args)
    aruco_detector = ArucoDetector()
    rclpy.spin(aruco_detector)
    aruco_detector.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
