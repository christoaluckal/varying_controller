import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Imu, NavSatFix
from geometry_msgs.msg import Twist
from message_filters import ApproximateTimeSynchronizer, Subscriber
from tf_transformations import euler_from_quaternion
from deltacan.msg import DeltaCan, Baseline
from collections import deque
import sys
import signal
import os
from tqdm import tqdm
# baseline_dir_deg

class Color:
    HEADER = '\033[95m'
    OKBLUE = '\033[94m'
    OKGREEN = '\033[92m'
    WARNING = '\033[93m'
    FAIL = '\033[91m'
    ENDC = '\033[0m'
    BOLD = '\033[1m'
    UNDERLINE = '\033[4m'

    @staticmethod
    def print_info(msg):
        string = f"{Color.OKBLUE}{Color.BOLD}{msg}{Color.ENDC}"
        return string
    
    @staticmethod
    def print_warning(msg):
        string = f"{Color.WARNING}{Color.BOLD}{msg}{Color.ENDC}"
        return string
    
    @staticmethod
    def print_error(msg):
        string = f"{Color.FAIL}{Color.BOLD}{msg}{Color.ENDC}"
        return string
    
    @staticmethod   
    def print_success(msg):
        string = f"{Color.OKGREEN}{Color.BOLD}{msg}{Color.ENDC}"
        return string

class SensorSyncNode(Node):
    def __init__(self):
        super().__init__('sensor_sync_node')

        self.declare_parameter('imu_topic', '/imu/data')
        self.declare_parameter('gps_topic', '/fix')
        self.declare_parameter('deltacan_topic','/deltacan')
        self.declare_parameter('baseline_topic', '/baseline')

        self.sub_imu = Subscriber(self, Imu, self.get_parameter('imu_topic').value)
        self.sub_gps = Subscriber(self, NavSatFix, self.get_parameter('gps_topic').value)
        self.sub_deltacan = Subscriber(self, DeltaCan, self.get_parameter('deltacan_topic').value)
        self.sub_baseline = Subscriber(self, Baseline, self.get_parameter('baseline_topic').value)

        self.queue = deque(maxlen=int(1e6))

        self.ts = ApproximateTimeSynchronizer(
            [self.sub_imu, self.sub_gps, self.sub_deltacan, self.sub_baseline], queue_size=10, slop=1.0)
        self.ts.registerCallback(self.sync_callback)

        self.get_logger().info("Sensor synchronization node started.")

    def sync_callback(self, imu_msg: Imu, gps_msg: NavSatFix, deltacan_msg: DeltaCan, baseline_msg: Baseline):
        # Convert GPS to UTM or Cartesian
        self.get_logger().info(f"Current Queue Length: {len(self.queue)}")

        t = gps_msg.header.stamp.sec + gps_msg.header.stamp.nanosec * 1e-9

        x = gps_msg.longitude
        y = gps_msg.latitude

        u_l = deltacan_msg.mlefttravelcmd
        u_r = deltacan_msg.mrighttravelcmd

        yaw = baseline_msg.baseline_dir_deg

        # IMU linear acceleration
        ax = imu_msg.linear_acceleration.x
        ay = imu_msg.linear_acceleration.y
        az = imu_msg.linear_acceleration.z

        data_row = [t, x, y, yaw, ax, ay, az, u_l, u_r]
        self.queue.append(data_row)
        # Save to file or publish if needed


    def destroy_node(self):
        curr_dir = os.path.dirname(os.path.abspath(__file__))
        csv_file_path = os.path.join(curr_dir, 'sensor_data.csv')
        self.get_logger().info(Color.print_info(f"Saving data to {csv_file_path}"))
        with open(csv_file_path, 'w') as f:
            col = f"t,x,y,yaw,ax,ay,az,u_l,u_r\n"
            f.write(col)
            for row in tqdm(self.queue):
                f.write(','.join(map(str, row)) + '\n')
        return

def main(args=None):
    rclpy.init(args=args)
    node = SensorSyncNode()

    def handle_shutdown(signum, frame):
        node.get_logger().info("Caught shutdown signal (Ctrl+C)")
        node.destroy_node()
        rclpy.shutdown()
        sys.exit(0)

    signal.signal(signal.SIGINT, handle_shutdown)

    try:
        rclpy.spin(node)
    except Exception as e:
        node.get_logger().error(f"Exception: {e}")
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
