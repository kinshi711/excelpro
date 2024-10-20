import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from geometry_msgs.msg import Twist
from cv_bridge import CvBridge
import cv2
import numpy as np

class LineFollower(Node):
    def __init__(self):
        super().__init__('line_follower')
        self.bridge = CvBridge()
        self.image_sub = self.create_subscription(Image, '/camera/image_raw', self.image_callback, 10)
        self.cmd_vel_pub = self.create_publisher(Twist, '/cmd_vel', 10)

        self.twist = Twist()
        self.lap_count = 0
        self.red_line_detected = False
        self.prev_red_line_detected = False

    def image_callback(self, data):
        # ROS2のイメージメッセージをOpenCV形式に変換
        cv_image = self.bridge.imgmsg_to_cv2(data, "bgr8")
        hsv = cv2.cvtColor(cv_image, cv2.COLOR_BGR2HSV)

        # 赤色の範囲を定義してマスクを作成
        lower_red = np.array([0, 120, 70])
        upper_red = np.array([10, 255, 255])
        mask1 = cv2.inRange(hsv, lower_red, upper_red)

        lower_red = np.array([170, 120, 70])
        upper_red = np.array([180, 255, 255])
        mask2 = cv2.inRange(hsv, lower_red, upper_red)

        red_mask = mask1 + mask2
        red_detected = np.sum(red_mask) > 0  # 赤い線が検出されたかどうか

        if red_detected:
            self.red_line_detected = True
        else:
            self.red_line_detected = False

        # デバッグ表示 (任意)
        cv2.imshow("Red Line Detection", red_mask)
        cv2.waitKey(1)

    def follow_line(self):
        timer_period = 0.1  # 100msごとに動作
        self.timer = self.create_timer(timer_period, self.timer_callback)

    def timer_callback(self):
        # 赤い線が検出された場合の動作
        if self.red_line_detected:
            self.twist.linear.x = 0.2  # 前進
            self.twist.angular.z = 0.0  # 直進
        else:
            self.twist.angular.z = 0.3  # 見失った場合は回転

        # 赤い線を通過したときに周回数を増加
        if self.red_line_detected and not self.prev_red_line_detected:
            self.lap_count += 1
            self.get_logger().info(f'Lap {self.lap_count} completed')

        # 3周したら停止
        if self.lap_count >= 3:
            self.twist.linear.x = 0.0
            self.twist.angular.z = 0.0
            self.get_logger().info('Three laps completed. Stopping robot.')
            self.cmd_vel_pub.publish(self.twist)
            self.destroy_timer(self.timer)
            return

        # ロボットの速度を更新
        self.cmd_vel_pub.publish(self.twist)
        self.prev_red_line_detected = self.red_line_detected

def main(args=None):
    rclpy.init(args=args)
    line_follower = LineFollower()
    line_follower.follow_line()
    rclpy.spin(line_follower)
    line_follower.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
