import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Imu
from sensor_msgs.msg import NavSatFix
from deltacan.msg import DeltaCan, Baseline
import numpy as np

class TestMsgPublisher(Node):
    def __init__(self):
        super().__init__('test_msg_publisher')

        self.declare_parameter('imu_topic', '/imu/data')
        self.declare_parameter('gps_topic', '/fix')
        self.declare_parameter('deltacan_topic','/deltacan')
        self.declare_parameter('baseline_topic', '/baseline')

        imu_rate = 100
        gps_rate = 10
        deltacan_rate = 50
        baseline_rate = 10

        self.imu_pub = self.create_publisher(Imu, self.get_parameter('imu_topic').value, 10)
        self.gps_pub = self.create_publisher(NavSatFix, self.get_parameter('gps_topic').value, 10)
        self.deltacan_pub = self.create_publisher(DeltaCan, self.get_parameter('deltacan_topic').value, 10)
        self.baseline_pub = self.create_publisher(Baseline, self.get_parameter('baseline_topic').value, 10)
        self.get_logger().info("Test message publisher node started.")


        self.imu_timer = self.create_timer(1.0 / imu_rate, self.publish_imu)    # 50 Hz
        self.gps_timer = self.create_timer(1.0 / gps_rate, self.publish_gps)    # 10 Hz
        self.deltacan_timer = self.create_timer(1.0 / deltacan_rate, self.publish_deltacan)
        self.baseline_timer = self.create_timer(1.0 / baseline_rate, self.publish_baseline)



    def publish_imu(self):
        imu_msg = Imu()
        imu_msg.header.stamp = self.get_clock().now().to_msg()
        self.imu_pub.publish(imu_msg)
        self.get_logger().debug("Published empty IMU")

    def publish_gps(self):
        gps_msg = NavSatFix()
        gps_msg.header.stamp = self.get_clock().now().to_msg()
        self.gps_pub.publish(gps_msg)
        self.get_logger().debug("Published empty GPS")

    def publish_deltacan(self):
        deltacan_msg = DeltaCan()
        deltacan_msg.header.stamp = self.get_clock().now().to_msg()
        deltacan_msg.mlefttravelcmd = np.random.uniform(-1.0, 1.0)
        deltacan_msg.mrighttravelcmd = np.random.uniform(-1.0, 1.0)
        self.deltacan_pub.publish(deltacan_msg)
        self.get_logger().debug("Published empty DeltaCan")

    def publish_baseline(self):
        baseline_msg = Baseline()
        baseline_msg.header.stamp = self.get_clock().now().to_msg()
        baseline_msg.baseline_dir_deg = np.random.uniform(-180.0, 180.0)
        self.baseline_pub.publish(baseline_msg)
        self.get_logger().debug("Published empty Baseline")

def main(args=None):
    rclpy.init(args=args)
    node = TestMsgPublisher()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.get_logger().info("Shutting down test publisher.")
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
