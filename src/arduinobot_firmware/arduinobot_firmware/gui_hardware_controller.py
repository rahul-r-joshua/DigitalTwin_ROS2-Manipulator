#!/usr/bin/env python3


import rclpy
from rclpy.node import Node
from sensor_msgs.msg import JointState
import serial
import time
import math
import threading
import os
from rclpy.qos import QoSProfile

class GUIHardwareController(Node):
    def __init__(self):
        super().__init__('gui_hardware_controller')

        self.declare_parameter('port', '/dev/ttyACM1')
        self.declare_parameter('baudrate', 115200)
        self.declare_parameter('update_rate', 20.0)

        self.port = self.get_parameter('port').value
        self.baudrate = self.get_parameter('baudrate').value
        self.update_rate = self.get_parameter('update_rate').value

        self.current_positions = {
            'joint_1': 90,
            'joint_2': 90,
            'joint_3': 180,
            'joint_4': 90
        }

        self.last_sent = {
            'joint_1': None,
            'joint_2': None,
            'joint_3': None,
            'joint_4': None
        }

        self.arduino = None
        self.arduino_lock = threading.Lock()
        self.arduino_warning_logged = False
        self.connect_arduino()

        self.joint_sub = self.create_subscription(
            JointState,
            '/joint_states',
            self.joint_state_callback,
            QoSProfile(depth=10)
        )

        self.joint_state_pub = self.create_publisher(
            JointState,
            '/joint_states',
            QoSProfile(depth=10)
        )

        self.timer = self.create_timer(
            1.0 / self.update_rate,
            self.send_to_arduino
        )

        self.get_logger().info('=' * 60)
        self.get_logger().info('🎮 GUI Hardware Controller Started')
        self.get_logger().info(f'   Port: {self.port} @ {self.baudrate} baud')
        self.get_logger().info(f'   Update Rate: {self.update_rate} Hz')
        self.get_logger().info('   Listening to /joint_states...')
        self.get_logger().info('=' * 60)

    def joint_state_callback(self, msg):
        for i, name in enumerate(msg.name):
            if name in self.current_positions and i < len(msg.position):
                rad = msg.position[i]
                if name == 'joint_1':
                    deg = int(math.degrees(rad) + 90)
                    self.current_positions[name] = max(0, min(180, deg))
                elif name == 'joint_2':
                    deg = int(round(-math.degrees(rad) + 90))
                    self.current_positions[name] = max(0, min(180, deg))
                elif name == 'joint_3':
                    deg = int(round(180 - math.degrees(rad)))
                    self.current_positions[name] = max(107, min(180, deg))
                elif name == 'joint_4':
                    deg = int(round(-math.degrees(rad) + 90))
                    self.current_positions[name] = max(90, min(180, deg))
        if not hasattr(self, '_first_msg_logged'):
            self.get_logger().info(f'📥 Receiving /joint_states messages')
            self._first_msg_logged = True

    def connect_arduino(self):
        if not os.path.exists(self.port):
            error_msg = f'\033[91m❌ ERROR: Port {self.port} does not exist! Please check Arduino connection.\033[0m'
            self.get_logger().error(error_msg)
            print(error_msg)
            self.get_logger().warn('⚠️  Continuing without Arduino connection...')
            self.arduino = None
            return

        try:
            import subprocess
            result = subprocess.run(['udevadm', 'info', '--name=' + self.port],
                                  capture_output=True, text=True, timeout=2)
            device_info = result.stdout.lower()

            is_arduino = any(keyword in device_info for keyword in
                           ['arduino', 'ch340', 'ftdi', 'usb-serial', '2341', '2a03', '1a86', '10c4', 'cp210', 'silicon labs'])

            if not is_arduino:
                warn_msg = (f'\033[93m⚠️  Port {self.port} does not match known Arduino/USB-serial identifiers.\n'
                           f'   Device info: Check with "udevadm info --name={self.port}"\n'
                           f'   Continuing anyway...\033[0m')
                self.get_logger().warn(warn_msg)
                print(warn_msg)
            else:
                self.get_logger().info(f'✓ Arduino/USB-serial device detected on {self.port}')

        except subprocess.TimeoutExpired:
            self.get_logger().warn(f'⚠️  Could not verify device type (timeout), proceeding anyway...')
        except FileNotFoundError:
            self.get_logger().warn(f'⚠️  udevadm not found, skipping device verification')
        except Exception as e:
            self.get_logger().warn(f'⚠️  Device verification failed: {e}, proceeding anyway...')

        try:
            self.arduino = serial.Serial(
                port=self.port,
                baudrate=self.baudrate,
                timeout=0.1,
                write_timeout=0.1
            )
            time.sleep(2.5)
            self.get_logger().info(f'✓ Arduino connected on {self.port}')

            time.sleep(0.5)
            with self.arduino_lock:
                self.arduino.write(b"b90,s90,e180,g90,")
                self.arduino.flush()

        except Exception as e:
            error_msg = f'\033[91m✗ Failed to connect Arduino: {e}\033[0m'
            self.get_logger().error(error_msg)
            print(error_msg)
            self.get_logger().warn('⚠️  Continuing without Arduino connection...')
            self.arduino = None

    def send_to_arduino(self):
        if not self.arduino:
            if not self.arduino_warning_logged:
                self.get_logger().warn('⚠️  Arduino not connected, skipping send')
                self.arduino_warning_logged = True
            return

        try:
            base_deg = self.current_positions['joint_1']
            shoulder_deg = self.current_positions['joint_2']
            elbow_deg = self.current_positions['joint_3']
            gripper_deg = self.current_positions['joint_4']

            threshold = 0.5
            changed = (self.position_changed('joint_1', base_deg, threshold) or
                self.position_changed('joint_2', shoulder_deg, threshold) or
                self.position_changed('joint_3', elbow_deg, threshold) or
                self.position_changed('joint_4', gripper_deg, threshold))

            if changed:
                self.publish_joint_states(base_deg, shoulder_deg, elbow_deg, gripper_deg)

                self.last_sent['joint_1'] = base_deg
                self.last_sent['joint_2'] = shoulder_deg
                self.last_sent['joint_3'] = elbow_deg
                self.last_sent['joint_4'] = gripper_deg

                if self.arduino:
                    try:
                        command = f"b{base_deg},s{shoulder_deg},e{elbow_deg},g{gripper_deg},"

                        with self.arduino_lock:
                            self.arduino.write(command.encode())
                            self.arduino.flush()

                        self.get_logger().info(
                            f'📤 → Arduino: B={base_deg}° S={shoulder_deg}° E={elbow_deg}° G={gripper_deg}°'
                        )

                    except Exception as e:
                        self.get_logger().error(f'✗ Send failed: {e}')
                else:
                    if not self.arduino_warning_logged:
                        self.get_logger().warn('⚠️  Arduino not connected, only updating RViz visualization')
                        self.arduino_warning_logged = True

        except Exception as e:
            self.get_logger().error(f'✗ Send failed: {e}')

    def position_changed(self, joint_name, new_deg, threshold):
        if self.last_sent[joint_name] is None:
            return True
        return abs(new_deg - self.last_sent[joint_name]) > threshold

    def publish_joint_states(self, base_deg, shoulder_deg, elbow_deg, gripper_deg):
        msg = JointState()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = ''
        msg.name = ['joint_1', 'joint_2', 'joint_3', 'joint_4']

        elbow_position = min(1.00, math.radians(180 - elbow_deg))

        msg.position = [
            math.radians(base_deg - 90),
            -math.radians(shoulder_deg - 90),
            elbow_position,
            -math.radians(gripper_deg - 90)
        ]
        msg.velocity = []
        msg.effort = []

        self.joint_state_pub.publish(msg)

def main(args=None):
    rclpy.init(args=args)
    controller = GUIHardwareController()

    try:
        rclpy.spin(controller)
    except KeyboardInterrupt:
        controller.get_logger().info('Shutting down...')
    finally:
        if controller.arduino:
            controller.arduino.close()
        controller.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
