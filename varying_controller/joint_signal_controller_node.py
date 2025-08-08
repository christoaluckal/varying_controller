import numpy as np
import matplotlib.pyplot as plt
import random
from rclpy.node import Node
from deltacan.msg import DeltaCan
from std_msgs.msg import Bool, Float64MultiArray
import os
import json
import time
import rclpy
import yaml
import pprint

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

class InteractiveSignalGenerator(Node):
    COMPONENT_MAP = {0: 'boom', 1: 'arm', 2: 'bucket'}

    def __init__(self):
        super().__init__('interactive_signal_generator_node')
        
        # Declare a parameter to receive the full path to the params YAML file.
        self.declare_parameter('params_file', rclpy.Parameter.Type.STRING)
        
        # Declare simple parameters.
        self.declare_parameter('deltacan_topic', '/deltacan')
        self.declare_parameter('safety_topic', '/kinematicsafety')
        self.declare_parameter('control_freq', 50.0)
        self.declare_parameter('noise', 0.0)
        
        # Get simple parameters
        self.deltacan_topic = self.get_parameter('deltacan_topic').value
        self.safety_topic = self.get_parameter('safety_topic').value
        self.control_freq = self.get_parameter('control_freq').value
        self.noise = self.get_parameter('noise').value

        # Load the complex dictionary from the YAML file using the path from the launch file
        self.SIGNAL_SETS = {}
        try:
            params_file_path = self.get_parameter('params_file').value
            
            self.get_logger().info(f"Loading signal sets from: {params_file_path}")
            
            with open(params_file_path, 'r') as f:
                params_data = yaml.safe_load(f)
                
                # Parsing Logic
                signal_sets_from_file = None
                for top_level_key in params_data:
                    if 'ros__parameters' in params_data[top_level_key]:
                        param_dict = params_data[top_level_key]['ros__parameters']
                        if 'signal_sets' in param_dict:
                            signal_sets_from_file = param_dict['signal_sets']
                            self.get_logger().info(f"Found 'signal_sets' under the key '{top_level_key}'.")
                            break
                
                if signal_sets_from_file is None:
                    raise Exception("Could not find 'signal_sets' within a 'ros__parameters' block in the YAML file.")

                self.SIGNAL_SETS = {int(k): v for k, v in signal_sets_from_file.items()}
                self.get_logger().info(f"Successfully loaded {len(self.SIGNAL_SETS)} signal sets from file.")

        except Exception as e:
            self.get_logger().fatal(f"FATAL: Failed to load or parse signal sets. Error: {e}")
            rclpy.shutdown()
            return

        self.is_safe_to_move = True
        self.timer = None
        self.boom_ys = np.array([])
        self.arm_ys = np.array([])
        self.bucket_ys = np.array([])
        self.total_steps = 0
        self.signal_index = 0
        self.deltacan_pub = self.create_publisher(DeltaCan, self.deltacan_topic, 10)
        self.safety_subscriber = self.create_subscription(
            Bool, self.safety_topic, self.safety_callback, 10)
        self.command_subscriber = self.create_subscription(
            Float64MultiArray, '/run_sequence', self.sequence_callback, 10)
        self.get_logger().info(Color.print_success("--- Interactive Controller Ready (Params Loaded) ---"))
        self.get_logger().info("Waiting for command on topic '/run_sequence'...")
        self.get_logger().info(
        'Example: ros2 topic pub --once /run_sequence std_msgs/msg/Float64MultiArray '
        '"{data: [0.0, 10.0, 2.0, 1.0, 10.0, 7.0, 2.0, 12.0, 14.0]}"'
        '\n  Format is a series of [ID, Duration, Set(0-14)] triplets. (IDs: 0=Boom, 1=Arm, 2=Bucket)')
        self.get_logger().info(Color.print_success('\n'+pprint.pformat(self.fmt_set())))


    def fmt_set(self):
        new_dict = {}
        for k,v in self.SIGNAL_SETS.items():
            vals = v.split(',')
            if vals[0]=='0.0':
                t = 'const'
                new_dict[k] = (t,vals[-1])
            elif vals[0]=='1.0':
                t = 'ramp'
                new_dict[k] = (t,vals[-2],vals[-1])
            elif vals[0]=='2.0':
                t = 'sine'
                new_dict[k] = (t,vals[-2],vals[-1])
            

        return new_dict

    def sequence_callback(self, msg):
        commands = list(msg.data)
        self.get_logger().info(Color.print_info(f"Received new command sequence: {commands}"))
        if self.timer is not None:
            self.timer.cancel()
            self.get_logger().info("Cancelled previous sequence.")
        self.boom_ys, self.arm_ys, self.bucket_ys = self.parse_and_generate(commands)
        self.total_steps = max(len(self.boom_ys), len(self.arm_ys), len(self.bucket_ys))
        self.signal_index = 0
        if self.total_steps > 0:
            self.get_logger().info(f"Generated new motion with {self.total_steps} steps. Starting now.")
            self.timer = self.create_timer(1.0 / self.control_freq, self.publish_signal)
        else:
            self.get_logger().warning("Command resulted in an empty sequence. No motion will occur.")

    def parse_and_generate(self, commands):
        component_signals = { 'boom': np.array([]), 'arm': np.array([]), 'bucket': np.array([]) }
        if len(commands) % 3 != 0:
            self.get_logger().error(f"Invalid command format. Must be triplets of (ID time set).")
            return np.array([]), np.array([]), np.array([])
        self.get_logger().info("--- Parsing Command Sequence ---")
        for i in range(0, len(commands), 3):
            try:
                raw_comp_id, duration, raw_set_num = commands[i], commands[i+1], commands[i+2]
                comp_id = int(raw_comp_id)
                if comp_id not in self.COMPONENT_MAP:
                    self.get_logger().error(Color.print_error(
                        f"Invalid component ID '{raw_comp_id}' in triplet {commands[i:i+3]}. Skipping. "
                        f"Please select from: 0 (boom), 1 (arm), 2 (bucket)."
                    ))
                    continue
                set_num = int(raw_set_num)
                if set_num not in self.SIGNAL_SETS:
                    self.get_logger().error(Color.print_error(
                        f"Invalid set number '{raw_set_num}' in triplet {commands[i:i+3]}. Skipping. "
                        f"Available sets are: {sorted(list(self.SIGNAL_SETS.keys()))}"
                    ))
                    continue
                comp_name = self.COMPONENT_MAP.get(comp_id)
                signal_template = self.SIGNAL_SETS.get(set_num)
                self.get_logger().info(f"  - Validated: {comp_name.capitalize()} | Set {set_num} | Duration {duration}s")
                signal_str = signal_template.format(time=duration)
                seq = [float(x) for x in signal_str.split(',')]
                _, y_signal = self.generate_signal_from_sequence(seq)
                if component_signals[comp_name].size > 0:
                    component_signals[comp_name] = np.concatenate((component_signals[comp_name], y_signal))
                else:
                    component_signals[comp_name] = y_signal
            except (ValueError, IndexError) as e:
                self.get_logger().error(f"Error parsing triplet {commands[i:i+3]}: {e}")
        self.get_logger().info("-----------------------------")
        return component_signals['boom'], component_signals['arm'], component_signals['bucket']

    def generate_signal_from_sequence(self, seq):
        sig_type, duration = int(seq[0]), seq[2]
        if sig_type == 0: return self.generate_const(seq[3], duration, self.noise)
        elif sig_type == 1: return self.generate_ramp(seq[3], seq[4], duration, self.noise)
        elif sig_type == 2:
            wave_freq = 1.0 / duration if duration > 0 else 1.0
            return self.generate_sine(seq[3], seq[4], duration, wave_freq, 0, self.noise)
        return np.array([]), np.array([])

    def safety_callback(self, msg):
        if self.is_safe_to_move != msg.data:
            self.is_safe_to_move = msg.data
            status = "engaged" if not msg.data else "released"
            self.get_logger().warning(Color.print_warning(f"Safety stop {status}!"))

    def publish_signal(self):
        if self.signal_index >= self.total_steps:
            self.get_logger().info(Color.print_info("All signals published."))
            self.timer.cancel()
            self.timer = None
            self.get_logger().info(Color.print_success('\n'+pprint.pformat(self.fmt_set())))
            return
        deltacan_msg = DeltaCan()
        deltacan_msg.header.stamp = self.get_clock().now().to_msg()
        if not self.is_safe_to_move:
            deltacan_msg.mboomcmd, deltacan_msg.marmcmd, deltacan_msg.mbucketcmd = 0.0, 0.0, 0.0
        else:
            deltacan_msg.mboomcmd = self.boom_ys[min(self.signal_index, len(self.boom_ys) - 1)] if len(self.boom_ys) > 0 else 0.0
            deltacan_msg.marmcmd = self.arm_ys[min(self.signal_index, len(self.arm_ys) - 1)] if len(self.arm_ys) > 0 else 0.0
            deltacan_msg.mbucketcmd = self.bucket_ys[min(self.signal_index, len(self.bucket_ys) - 1)] if len(self.bucket_ys) > 0 else 0.0
            self.signal_index += 1
        self.deltacan_pub.publish(deltacan_msg)

    def generate_const(self, value, duration, noise=0.0):
        dt = 1 / self.control_freq
        t = np.arange(0, duration, dt)
        if len(t) == 0: return np.array([]), np.array([])
        y = np.full_like(t, fill_value=value)
        if noise > 0: y += np.random.uniform(-noise, noise, size=y.shape); y = np.clip(y, -1, 1)
        return t, y

    def generate_ramp(self, min_range=0, max_range=1, duration=10, noise=0.02):
        dt = 1 / self.control_freq
        t = np.arange(0, duration, dt)
        if len(t) == 0: return np.array([]), np.array([])
        y = np.linspace(min_range, max_range, len(t))
        if noise > 0: y += np.random.uniform(-noise, noise, size=y.shape); y = np.clip(y, -1, 1)
        return t, y

    def generate_sine(self, min_range=0, max_range=1, duration=10, wave_freq=1, phase=0, noise=0.02):
        dt = 1 / self.control_freq
        t = np.arange(0, duration, dt)
        if len(t) == 0: return np.array([]), np.array([])
        y = np.sin(2 * np.pi * wave_freq * t + phase)
        amplitude = (max_range - min_range) / 2
        offset = (max_range + min_range) / 2
        y = y * amplitude + offset
        if noise > 0: y += np.random.uniform(-noise, noise, size=y.shape); y = np.clip(y, -1, 1)
        return t, y

def main(args=None):
    rclpy.init(args=args)
    node = InteractiveSignalGenerator()
    try:
        if rclpy.ok():
            rclpy.spin(node)
    except KeyboardInterrupt:
        node.get_logger().info(Color.print_warning("Signal generation interrupted by user."))
    finally:
        if 'node' in locals() and rclpy.ok():
             if node.timer:
                node.timer.cancel()
             node.destroy_node()
             rclpy.shutdown()

if __name__ == '__main__':
    main()