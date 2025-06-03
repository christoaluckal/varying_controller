import numpy as np
import matplotlib.pyplot as plt
import random
from rclpy.node import Node
from deltacan.msg import DeltaCan
import os
import json
import time
import rclpy

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

class SignalGenerator(Node):

    def __init__(self):
        super().__init__('generate_seq_signal_node')
        self.declare_parameter('deltacan_topic', '/deltacan')
        self.declare_parameter('control_freq', 10.0)
        self.declare_parameter('pause_duration', 5.0)
        self.declare_parameter('noise', 0.0)
        self.declare_parameter('sequence', rclpy.Parameter.Type.STRING_ARRAY)

        self.deltacan_topic = self.get_parameter('deltacan_topic').value
        self.control_freq = self.get_parameter('control_freq').value
        self.pause_duration = self.get_parameter('pause_duration').value
        self.sequence = self.get_parameter('sequence').value
        self.noise = self.get_parameter('noise').value

        self.sequences = [list(map(float, s.split(','))) for s in self.sequence]
        self.curricula_size = len(self.sequences)

        param_str = f"""
        ****************************************
        DeltaCan Topic: {self.deltacan_topic}
        Control Frequency: {self.control_freq} Hz
        Pause Duration: {self.pause_duration} seconds
        Sequence: {self.sequences}
        ****************************************
        """
        self.get_logger().info(Color.print_warning(param_str))

        self.signal_index = 0

        if int(self.sequences[0][0]) == 0:
            self.curricula = self.generate_const_curriculum()
        elif int(self.sequences[0][0]) == 1:
            self.curricula = self.generate_ramp_curriculum()
        else:
            self.curricula = self.generate_sine_curriculum()
        self.xs = self.curricula[0]
        self.ys_l = self.curricula[1]
        self.ys_r = self.curricula[2]

        self.xs = np.concatenate(self.xs)
        self.ys_l = np.concatenate(self.ys_l)
        self.ys_r = np.concatenate(self.ys_r)

        self.get_logger().info(Color.print_info(f"Generated {len(self.xs)} time points and {len(self.ys_l)} signal values."))

        
        self.curricula_info = self.curricula[3]
        self.get_logger().info(Color.print_info("Signal generator node started."))
        for i, curriculum in enumerate(self.curricula_info):
            self.get_logger().info(Color.print_info(f"Curriculum {i+1}: {curriculum}"))
 
        curr_file_path = os.path.dirname(os.path.abspath(__file__))
        self.curriculum_file = os.path.join(curr_file_path, 'curriculum.csv')

        with open(self.curriculum_file, 'w') as f:
            f.write("t,y_l,y_r\n")
            for i in range(len(self.xs)):
                f.write(f"{self.xs[i]},{self.ys_l[i]},{self.ys_r[i]}\n")
        self.get_logger().info(Color.print_success(f"Curriculum saved to {self.curriculum_file}"))

        self.deltacan_pub = self.create_publisher(DeltaCan, self.deltacan_topic, 10)
        
        self.timer = self.create_timer(1.0 / self.control_freq, self.publish_signal)
        
    def publish_signal(self):
        if self.signal_index >= len(self.xs):
            self.get_logger().info(Color.print_info("All signals published."))
            return
        
        t = self.xs[self.signal_index]
        y_l = self.ys_l[self.signal_index]
        y_r = self.ys_r[self.signal_index]

        deltacan_msg = DeltaCan()
        deltacan_msg.header.stamp = self.get_clock().now().to_msg()
        deltacan_msg.mlefttravelcmd = y_l
        deltacan_msg.mrighttravelcmd = y_r
        self.deltacan_pub.publish(deltacan_msg)

        # self.get_logger().info(Color.print_success(f"Published signal {self.signal_index + 1} at time {t[-1]} with value {y[-1]}"))
        self.signal_index += 1
        

    def generate_ramp(self, min_range=0, max_range=1, duration=10, control_freq=10, noise=0.02):
        """
        Generate a ramp signal.
        
        Parameters:
        - min_range: Minimum value of the ramp.
        - max_range: Maximum value of the ramp.
        - duration: Duration of the ramp in seconds.
        - freq: Frequency of the control signal.
        
        Returns:
        - t: Time vector.
        - y: Ramp signal values.
        """
        dt = 1 / self.control_freq
        t = np.arange(0, duration, dt)
        y = np.linspace(min_range, max_range, len(t))
        
        if noise > 0:
            y += np.random.uniform(-noise, noise, size=y.shape)
            y = np.clip(y, -1, 1)

        return t, y


    def generate_sine(self, min_range=0, max_range=1, duration=10, control_freq=10, wave_freq=1, phase=0, noise=0.02):
        """
        Generate a sine wave signal.

        Parameters:
        - min_range: Minimum value of the sine wave.
        - max_range: Maximum value of the sine wave.
        - duration: Duration of the sine wave in seconds.
        - control_freq: Frequency of the control signal (Hz).
        - wave_freq: Frequency of the sine wave (Hz).
        - phase: Phase shift of the sine wave in radians.
        - noise: Amplitude of uniform random noise to add.

        Returns:
        - t: Time vector.
        - y: Sine wave signal values within [min_range, max_range].
        """
        dt = 1 / self.control_freq
        t = np.arange(0, duration, dt)
        y = np.sin(2 * np.pi * wave_freq * t + phase)
        amplitude = (max_range - min_range) / 2
        offset = (max_range + min_range) / 2
        y = y * amplitude + offset
        if noise > 0:
            y += np.random.uniform(-noise, noise, size=y.shape)
            y = np.clip(y, -1, 1)

        return t, y



    def generate_const_curriculum(self):
        xs = []
        ys_l = []
        ys_r = []
        current_t = 0
        curricula = []
        duration = self.pause_duration
        t, y = self.generate_ramp(min_range=0, max_range=0, duration=duration, noise=0)
        t = t + current_t
        current_t += duration
        xs.append(t)
        ys_l.append(y)
        ys_r.append(y)
        for i in range(self.curricula_size):
            print(self.sequences[i])
            sig_type, inv_flag, duration, value = self.sequences[i]
            if int(sig_type) == 0:  # Ramp
                inv_flag = bool(inv_flag)
                t, y = self.generate_ramp(min_range=value, max_range=value, duration=duration, noise=self.noise)
                t = t + current_t
                current_t += duration
                xs.append(t)
                ys_l.append(y)
                ys_r.append(y if not inv_flag else -y)
                curricula.append({
                    'type': 'ramp',
                    'dir': 'straight' if not inv_flag else 'inverse',
                    't': current_t,
                    'value': value,
                    'duration': duration
                })

            duration = self.pause_duration
            t, y = self.generate_ramp(min_range=0, max_range=0, duration=duration, noise=0)
            t = t + current_t
            current_t += duration
            xs.append(t)
            ys_l.append(y)
            ys_r.append(y)
        
        return [xs, ys_l,ys_r, curricula]

    def generate_ramp_curriculum(self):
        xs = []
        ys_l = []
        ys_r = []
        current_t = 0
        curricula = []
        duration = self.pause_duration
        t, y = self.generate_ramp(min_range=0, max_range=0, duration=duration, noise=0)
        t = t + current_t
        current_t += duration
        xs.append(t)
        ys_l.append(y)
        ys_r.append(y)
        
        for i in range(self.curricula_size):
            sig_type, inv_flag, duration, min_val, max_val = self.sequences[i]
            if int(sig_type) == 1:
                inv_flag = bool(inv_flag)
                t, y = self.generate_ramp(min_range=min_val, max_range=max_val, duration=duration, noise=self.noise)
                t = t + current_t
                current_t += duration
                xs.append(t)
                ys_l.append(y)
                ys_r.append(y if not inv_flag else -y)
                curricula.append({
                    'type': 'ramp',
                    'dir': 'straight' if not inv_flag else 'inverse',
                    't': current_t,
                    'min_val': min_val,
                    'max_val': max_val,
                    'duration': duration
                })
            duration = self.pause_duration
            t, y = self.generate_ramp(min_range=0, max_range=0, duration=duration, noise=0)
            t = t + current_t
            current_t += duration
            xs.append(t)
            ys_l.append(y)
            ys_r.append(y)
        return [xs, ys_l, ys_r, curricula]
    
    def generate_sine_curriculum(self):
        xs = []
        ys_l = []
        ys_r = []
        current_t = 0
        curricula = []
        duration = self.pause_duration
        t, y = self.generate_ramp(min_range=0, max_range=0, duration=duration, noise=0)
        t = t + current_t
        current_t += duration
        xs.append(t)
        ys_l.append(y)
        ys_r.append(y)

        for i in range(self.curricula_size):
            sig_type, inv_flag, duration, min_val, max_val = self.sequences[i]
            wave_freq = 1.0/ duration if duration > 0 else 1.0
            phase = 0.0
            if int(sig_type) == 2:
                inv_flag = bool(inv_flag)
                t, y = self.generate_sine(min_range=min_val, max_range=max_val, duration=duration,
                                          wave_freq=wave_freq, phase=phase, noise=self.noise)
                t = t + current_t
                current_t += duration
                xs.append(t)
                ys_l.append(y)
                ys_r.append(y if not inv_flag else -y)
                curricula.append({
                    'type': 'sine',
                    'dir': 'straight' if not inv_flag else 'inverse',
                    't': current_t,
                    'min_val': min_val,
                    'max_val': max_val,
                    'wave_freq': wave_freq,
                    'phase': phase,
                    'duration': duration
                })
            duration = self.pause_duration
            t, y = self.generate_ramp(min_range=0, max_range=0, duration=duration, noise=0)
            t = t + current_t
            current_t += duration
            xs.append(t)
            ys_l.append(y)
            ys_r.append(y)

        return [xs, ys_l, ys_r, curricula]

    def save_control(self):
        curr_file_path = os.path.dirname(os.path.abspath(__file__))
        csv_path = os.path.join(curr_file_path, 'curriculum.csv')
        self.get_logger().info(Color.print_success(f"Saving curriculum to {csv_path}"))
        with open(csv_path, 'w') as f:
            f.write("t,y\n")
            for i in range(len(self.xs)):
                f.write(f"{self.xs[i]},{self.ys[i]}\n")
        self.get_logger().info(Color.print_success(f"Curriculum saved to {csv_path}"))

def main(args=None):
    import rclpy
    rclpy.init(args=args)
    node = SignalGenerator()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.get_logger().info(Color.print_warning("Signal generation interrupted by user."))
        node.save_control()

    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()