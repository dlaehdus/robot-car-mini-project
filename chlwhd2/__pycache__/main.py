import time
import threading
import math
import serial
import struct

import rclpy
from rclpy.node import Node
from std_msgs.msg import String
from dynamixel_sdk import PortHandler, PacketHandler


class KeyboardController:
    def __init__(self):
        self.keys_pressed = set()
        self.velocity = 0
        self.steering = 0
        self.robot_exit = False
        self.VELOCITY_STEP = 10
        self.STEERING_STEP = 5
        self.VELOCITY_MAX = 100
        self.VELOCITY_MIN = -100
        self.STEERING_MAX = 60
        self.STEERING_MIN = -60
        self.RETURN_VELOCITY = 5
        self.RETURN_STEERING = 10

    def process_key_event(self, event: str):
        if event.startswith("press:"):
            key = event[len("press:"):].strip().lower()
            if key == "backspace":
                self.robot_exit = True
            else:
                self.keys_pressed.add(key)
        elif event.startswith("release:"):
            key = event[len("release:"):].strip().lower()
            self.keys_pressed.discard(key)

    def start(self):
        self.control_thread = threading.Thread(target=self.update_controls, daemon=True)
        self.control_thread.start()

    def update_controls(self):
        while not self.robot_exit:
            if 'w' in self.keys_pressed:
                self.velocity = min(self.velocity + self.VELOCITY_STEP, self.VELOCITY_MAX)
            if 's' in self.keys_pressed:
                self.velocity = max(self.velocity - self.VELOCITY_STEP, self.VELOCITY_MIN)
            if 'w' not in self.keys_pressed and 's' not in self.keys_pressed:
                if self.velocity > 0:
                    self.velocity = max(self.velocity - self.RETURN_VELOCITY, 0)
                elif self.velocity < 0:
                    self.velocity = min(self.velocity + self.RETURN_VELOCITY, 0)
            if 'd' in self.keys_pressed:
                self.steering = min(self.steering + self.STEERING_STEP, self.STEERING_MAX)
            if 'a' in self.keys_pressed:
                self.steering = max(self.steering - self.STEERING_STEP, self.STEERING_MIN)
            if 'd' not in self.keys_pressed and 'a' not in self.keys_pressed:
                if self.steering > 0:
                    self.steering = max(self.steering - self.RETURN_STEERING, 0)
                elif self.steering < 0:
                    self.steering = min(self.steering + self.RETURN_STEERING, 0)
            if 'r' in self.keys_pressed:
                self.velocity = 0
            time.sleep(0.05)

class SteeringMotorController:
    def __init__(self, port_name='/dev/ttyACM4', baud_rate=1000000, steering_ids=(1, 2, 3, 4)):
        # steering_ids 순서: [front_right, front_left, rear_right, rear_left]
        self.port = PortHandler(port_name)
        self.packet = PacketHandler(2.0)
        if not (self.port.openPort() and self.port.setBaudRate(baud_rate)):
            print("❌ 포트를 열거나 보드레이트 설정에 실패했습니다.")
            raise Exception("포트 연결 실패")
        self.steering_ids = steering_ids
        self.INIT_POSITION = 2047
        for _id in self.steering_ids:
            self.packet.write1ByteTxRx(self.port, _id, 64, 0)
            self.packet.write1ByteTxRx(self.port, _id, 11, 3)
            self.packet.write1ByteTxRx(self.port, _id, 64, 1)
            self.packet.write4ByteTxRx(self.port, _id, 116, self.INIT_POSITION)
        self.ROBOT_WIDTH = 305.96
        self.ROBOT_LENGTH = 365.5
        self.front_wheelbase = self.ROBOT_LENGTH / 2
        self.rear_wheelbase = self.ROBOT_LENGTH / 2

    def steering_to_position(self, angle):
        # 45°당 512 단위로 변환 (다이나믹셀의 중립값 2047에서 오프셋)
        return self.INIT_POSITION + int((angle / 45.0) * 512)

    def update_steering(self, steering_angle):
        if steering_angle == 0:
            angles = [0, 0, 0, 0]
        else:
            direction = 1 if steering_angle > 0 else -1
            input_angle = abs(steering_angle)
            inner_angle_rad = math.radians(input_angle)
            R = self.front_wheelbase / math.tan(inner_angle_rad) + (self.ROBOT_WIDTH / 2)
            outer_front_angle = math.degrees(math.atan(self.front_wheelbase / (R + self.ROBOT_WIDTH / 2)))
            inner_rear_angle = math.degrees(math.atan(self.rear_wheelbase / (R - self.ROBOT_WIDTH / 2)))
            outer_rear_angle = math.degrees(math.atan(self.rear_wheelbase / (R + self.ROBOT_WIDTH / 2)))
            if direction > 0:
                front_left = input_angle
                front_right = outer_front_angle
                rear_left = - inner_rear_angle
                rear_right = - outer_rear_angle
            else:
                front_right = input_angle
                front_left = outer_front_angle
                rear_right = inner_rear_angle
                rear_left = outer_rear_angle
                front_right = -front_right
                front_left = -front_left
            angles = [front_right, front_left, rear_right, rear_left]
        target_positions = [self.steering_to_position(a) for a in angles]
        for motor_id, pos in zip(self.steering_ids, target_positions):
            self.packet.write4ByteTxRx(self.port, motor_id, 116, pos)
        return angles

    def shutdown(self):
        for _id in self.steering_ids:
            self.packet.write4ByteTxRx(self.port, _id, 116, self.INIT_POSITION)
            self.packet.write1ByteTxRx(self.port, _id, 64, 0)
        self.port.closePort()

class InWheelMotorController:
    def __init__(self, port_list=("/dev/ttyACM0", "/dev/ttyACM1", "/dev/ttyACM2", "/dev/ttyACM3"),
                 velocity_ids=(0x00, 0x01, 0x02, 0x03)):
        self.port_list = port_list
        self.velocity_ids = velocity_ids
        self.serial_connections = []
        for port in self.port_list:
            ser = self.connect_serial(port)
            self.serial_connections.append(ser)

    def connect_serial(self, port, baudrate=115200, timeout=1):
        try:
            ser = serial.Serial(
                port=port,
                baudrate=baudrate,
                parity=serial.PARITY_NONE,
                stopbits=serial.STOPBITS_ONE,
                bytesize=serial.EIGHTBITS,
                timeout=timeout
            )
            return ser
        except serial.SerialException as e:
            print(f"❌ {port} 연결 실패: {e}")
            raise

    @staticmethod
    def decimal_to_hex_bytes(decimal):
        hex_string = format(struct.unpack('>H', struct.pack('>h', int(decimal)))[0], 'x').zfill(4)
        return int(hex_string[:2], 16), int(hex_string[2:], 16)

    @staticmethod
    def calculate_crc(data):
        if not isinstance(data, bytes):
            data = bytes(data)
        CRC8_MAXIM_table = (
            0x00, 0x5e, 0xbc, 0xe2, 0x61, 0x3f, 0xdd, 0x83,
            0xc2, 0x9c, 0x7e, 0x20, 0xa3, 0xfd, 0x1f, 0x41,
            0x9d, 0xc3, 0x21, 0x7f, 0xfc, 0xa2, 0x40, 0x1e,
            0x5f, 0x01, 0xe3, 0xbd, 0x3e, 0x60, 0x82, 0xdc,
            0x23, 0x7d, 0x9f, 0xc1, 0x42, 0x1c, 0xfe, 0xa0,
            0xe1, 0xbf, 0x5d, 0x03, 0x80, 0xde, 0x3c, 0x62,
            0xbe, 0xe0, 0x02, 0x5c, 0xdf, 0x81, 0x63, 0x3d,
            0x7c, 0x22, 0xc0, 0x9e, 0x1d, 0x43, 0xa1, 0xff,
            0x46, 0x18, 0xfa, 0xa4, 0x27, 0x79, 0x9b, 0xc5,
            0x84, 0xda, 0x38, 0x66, 0xe5, 0xbb, 0x59, 0x07,
            0xdb, 0x85, 0x67, 0x39, 0xba, 0xe4, 0x06, 0x58,
            0x19, 0x47, 0xa5, 0xfb, 0x78, 0x26, 0xc4, 0x9a,
            0x65, 0x3b, 0xd9, 0x87, 0x04, 0x5a, 0xb8, 0xe6,
            0xa7, 0xf9, 0x1b, 0x45, 0xc6, 0x98, 0x7a, 0x24,
            0xf8, 0xa6, 0x44, 0x1a, 0x99, 0xc7, 0x25, 0x7b,
            0x3a, 0x64, 0x86, 0xd8, 0x5b, 0x05, 0xe7, 0xb9,
            0x8c, 0xd2, 0x30, 0x6e, 0xed, 0xb3, 0x51, 0x0f,
            0x4e, 0x10, 0xf2, 0xac, 0x2f, 0x71, 0x93, 0xcd,
            0x11, 0x4f, 0xad, 0xf3, 0x70, 0x2e, 0xcc, 0x92,
            0xd3, 0x8d, 0x6f, 0x31, 0xb2, 0xec, 0x0e, 0x50,
            0xaf, 0xf1, 0x13, 0x4d, 0xce, 0x90, 0x72, 0x2c,
            0x6d, 0x33, 0xd1, 0x8f, 0x0c, 0x52, 0xb0, 0xee,
            0x32, 0x6c, 0x8e, 0xd0, 0x53, 0x0d, 0xef, 0xb1,
            0xf0, 0xae, 0x4c, 0x12, 0x91, 0xcf, 0x2d, 0x73,
            0xca, 0x94, 0x76, 0x28, 0xab, 0xf5, 0x17, 0x49,
            0x08, 0x56, 0xb4, 0xea, 0x69, 0x37, 0xd5, 0x8b,
            0x57, 0x09, 0xeb, 0xb5, 0x36, 0x68, 0x8a, 0xd4,
            0x95, 0xcb, 0x29, 0x77, 0xf4, 0xaa, 0x48, 0x16,
            0xe9, 0xb7, 0x55, 0x0b, 0x88, 0xd6, 0x34, 0x6a,
            0x2b, 0x75, 0x97, 0xc9, 0x4a, 0x14, 0xf6, 0xa8,
            0x74, 0x2a, 0xc8, 0x96, 0x15, 0x4b, 0xa9, 0xf7,
            0xb6, 0xe8, 0x0a, 0x54, 0xd7, 0x89, 0x6b, 0x35
        )
        crc = 0x00
        for byte in data:
            index = (crc ^ int(byte)) & 0xFF
            crc = CRC8_MAXIM_table[index]
        return crc

    @staticmethod
    def send_data(ser, data):
        ser.write(data)

    def set_velocity(self, ser, ID, speed):
        speed_H, speed_L = self.decimal_to_hex_bytes(speed)
        data_temp = bytes([
            int(ID) & 0xFF,
            0x64,
            int(speed_H) & 0xFF,
            int(speed_L) & 0xFF,
            0, 0, 0, 0, 0
        ])
        crc = self.calculate_crc(data_temp)
        data = data_temp + bytes([crc])
        self.send_data(ser, data)

    def brake(self, ser, ID):
        data_temp = bytes([
            int(ID) & 0xFF,
            0x64,
            0, 0, 0, 0, 0,
            0xFF, 0
        ])
        crc = self.calculate_crc(data_temp)
        data = data_temp + bytes([crc])
        self.send_data(ser, data)

    def set_velocity_all(self, velocity):
        for i, (ser, motor_id) in enumerate(zip(self.serial_connections, self.velocity_ids)):
            target_velocity = velocity if i % 2 == 0 else -velocity
            self.set_velocity(ser, motor_id, target_velocity)

    def shutdown(self):
        for ser, motor_id in zip(self.serial_connections, self.velocity_ids):
            self.brake(ser, motor_id)
            ser.close()

class MotorControlNode(Node):
    def __init__(self):
        super().__init__('motor_control_node')
        self.publisher = self.create_publisher(String, 'motor_status', 10)
        self.subscription = self.create_subscription(String, 'keyboard_input', self.keyboard_callback, 10)
        self.keyboard_ctrl = KeyboardController()
        self.keyboard_ctrl.start()
        self.inwheel_ctrl = InWheelMotorController(port_list=["/dev/ttyACM0", "/dev/ttyACM1", "/dev/ttyACM2", "/dev/ttyACM3"])
        self.steering_ctrl = SteeringMotorController(port_name='/dev/ttyACM4', steering_ids=(1,2,3,4))
        self.timer = self.create_timer(0.01, self.control_loop)

    def keyboard_callback(self, msg: String):
        self.keyboard_ctrl.process_key_event(msg.data)

    def control_loop(self):
        self.inwheel_ctrl.set_velocity_all(self.keyboard_ctrl.velocity)
        angles = self.steering_ctrl.update_steering(self.keyboard_ctrl.steering)
        status = (f"속도: {self.keyboard_ctrl.velocity} | "
                  f"FR: {angles[0]:.1f}°, FL: {angles[1]:.1f}°, "
                  f"RR: {angles[2]:.1f}°, RL: {angles[3]:.1f}° "
                  f"(steering: {self.keyboard_ctrl.steering}°)")
        self.publisher.publish(String(data=status))
        self.get_logger().info(status)

    def shutdown(self):
        self.steering_ctrl.shutdown()
        self.inwheel_ctrl.shutdown()

def main():
    rclpy.init()
    node = MotorControlNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.shutdown()
        node.destroy_node()
        rclpy.shutdown()

if __name__ == "__main__":
    main()
