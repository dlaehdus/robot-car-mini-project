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
    def __init__(self, port_name='/dev/ttyACM0', baud_rate=1000000, steering_ids=(0, 1)):
        self.port = PortHandler(port_name)
        self.packet = PacketHandler(2.0)
        if not (self.port.openPort() and self.port.setBaudRate(baud_rate)):
            print(" 포트를 열거나 보드레이트 설정에 실패했습니다.")
            raise Exception("포트 연결 실패")
        self.steering_ids = steering_ids
        self.INIT_POSITION = 2048
        for _id in self.steering_ids:
            self.packet.write1ByteTxRx(self.port, _id, 64, 0)
            self.packet.write1ByteTxRx(self.port, _id, 11, 3)
            self.packet.write1ByteTxRx(self.port, _id, 64, 1)
            self.packet.write4ByteTxRx(self.port, _id, 116, self.INIT_POSITION)
        self.ROBOT_WIDTH = 347
        self.ROBOT_LENGTH = 450
        self.front_wheelbase = self.ROBOT_LENGTH / 2
        self.rear_wheelbase = self.ROBOT_LENGTH / 2
        
        # 아크만 기하학을 위한 속도와 조향 각도 저장
        self.velocity = 0
        self.steering = 0

    def steering_to_position(self, angle):
        return self.INIT_POSITION + int((angle / 45.0) * 512)

    def compute_ackermann(self):
        vel = self.velocity
        steer = self.steering

        if steer == 0:                                              # 만약에 회전하지 않고 직진과 후진만 하는 상황이라면
            wheel_speeds = [vel, vel, vel, vel]                     # 바퀴 속도 리스트 생성: 모든 4개 바퀴(앞왼쪽 FL, 앞오른쪽 FR, 뒤왼쪽 RL, 뒤오른쪽 RR)에 vel 값을 동일하게 설정
            steering_angles = [0.0, 0.0]                            # 조향 각도 리스트 생성: 앞바퀴 2개(왼쪽 left, 오른쪽 right)에 0.0을 설정
        else:                                                       # 회전하는 상황이면 즉 선회상태이면
            direction = 1 if steer > 0 else -1                      # steer가 양수이면 direction=1(오른쪽 선회), 음수이면 -1(왼쪽 선회)으로 설정
            
            # 회전반경 계산 코드
            input_angle = abs(steer)                                # 입력 각도의 절대값을 input_angle에 저장
            inner_angle_rad = math.radians(input_angle)             # input_angle을 라디안 단위로 변환
            R = (self.ROBOT_LENGTH / math.tan(inner_angle_rad)) + (self.ROBOT_WIDTH / 2)                       # 선회 반경 R 계산 (아크만 기하학 공식.)
            outer_front_angle = math.degrees(math.atan(self.ROBOT_LENGTH / (R + self.ROBOT_WIDTH / 2)))      # 외측 앞바퀴 각도 계산: atan 함수로 각도를 구하고 degrees로 변환
            
            # 속도 계산 코드 각속도를 이용함
            w = (vel/R)
            r_inner_front = math.sqrt((R - (self.ROBOT_WIDTH / 2))**2 + (self.ROBOT_LENGTH)**2)
            r_outer_front = math.sqrt((R + (self.ROBOT_WIDTH / 2))**2 + (self.ROBOT_LENGTH)**2)
            r_inner_back = (R - (self.ROBOT_WIDTH / 2))
            r_outer_back = (R + (self.ROBOT_WIDTH / 2))
            
            v_inner_front = (w * r_inner_front)
            v_outer_front = (w * r_outer_front)
            v_inner_back = (w * r_inner_back)
            v_outer_back = (w * r_outer_back)

            if direction > 0:                                       # direction이 양수(오른쪽 선회)이면 아래 코드를 실행
                front_right = input_angle                           # 오른쪽 앞바퀴 각도를 input_angle(내측 각도)로 설정
                front_left = outer_front_angle                      # 왼쪽 앞바퀴 각도를 outer_front_angle(외측 각도)로 설정
            else:                                                   # direction이 음수(왼쪽 선회)이면 아래 코드를 실행
                front_left = input_angle                            # 왼쪽 앞바퀴 각도를 input_angle(내측 각도)로 설정
                front_right = outer_front_angle                     # 오른쪽 앞바퀴 각도를 outer_front_angle(외측 각도)로 설정
                front_left = -front_left                            # 왼쪽 각도를 음수로 변환합니다. (왼쪽 선회 방향 반영.)
                front_right = -front_right                          # 오른쪽 각도를 음수로 변환합니다. (왼쪽 선회 방향 반영.)
            steering_angles = [front_left, front_right]             # 조향 각도 리스트 생성: [왼쪽, 오른쪽] 순서로 저장

            if direction > 0:                                                               # 조건 검사: 오른쪽 선회이면 아래 코드를 실행
                wheel_speeds = [v_inner_front, v_outer_front, v_inner_back, v_outer_back]   # 바퀴 속도 리스트: [내측, 외측, 내측, 외측] 순서로 설정 (FL 내측, FR 외측 등).
            else:                                                                           # 왼쪽 선회이면 아래 코드를 실행
                wheel_speeds = [v_outer_front, v_inner_front, v_outer_back, v_inner_back]   # 바퀴 속도 리스트: [외측, 내측, 외측, 내측] 순서로 설정 (FL 외측, FR 내측 등).
        return wheel_speeds, steering_angles

    def update_steering(self, steering_angle, velocity):
        # 속도와 조향 각도 업데이트
        self.steering = steering_angle
        self.velocity = velocity
        
        # 아크만 기하학 계산
        wheel_speeds, steering_angles = self.compute_ackermann()
        
        # 조향 모터 제어 (기존 방식과 동일하지만 steering_angles 사용)
        target_positions = [self.steering_to_position(a) for a in steering_angles]
        for motor_id, pos in zip(self.steering_ids, target_positions):
            self.packet.write4ByteTxRx(self.port, motor_id, 116, pos)
        
        return wheel_speeds, steering_angles

    def shutdown(self):
        for _id in self.steering_ids:
            self.packet.write4ByteTxRx(self.port, _id, 116, self.INIT_POSITION)
            self.packet.write1ByteTxRx(self.port, _id, 64, 0)
        self.port.closePort()


class InWheelMotorController:
    def __init__(self, port_list=("/dev/ttyACM1", "/dev/ttyACM2", "/dev/ttyACM3", "/dev/ttyACM4"),
                 velocity_ids=(0x01, 0x02, 0x03, 0x04)):
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
            print(f"{port} 연결 실패: {e}")
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
        data_temp = bytes([int(ID) & 0xFF,0x64,int(speed_H) & 0xFF,int(speed_L) & 0xFF,0, 0, 0, 0, 0])
        crc = self.calculate_crc(data_temp)
        data = data_temp + bytes([crc])
        self.send_data(ser, data)

    def brake(self, ser, ID):
        data_temp = bytes([int(ID) & 0xFF,0x64,0, 0, 0, 0, 0,0xFF, 0])
        crc = self.calculate_crc(data_temp)
        data = data_temp + bytes([crc])
        self.send_data(ser, data)

    def set_velocity_individual(self, wheel_speeds):
        """
        각 바퀴에 개별 속도를 설정하는 메서드
        wheel_speeds: [FL, FR, RL, RR] 순서의 속도 리스트
        """
        for i, (ser, motor_id) in enumerate(zip(self.serial_connections, self.velocity_ids)):
            # 바퀴 순서에 따른 속도 설정
            target_velocity = wheel_speeds[i]
            # 모터 방향 보정 (왼쪽 바퀴는 반대 방향)
            if i % 2 == 0:  # FL, RL (왼쪽 바퀴, 인덱스 0, 2)
                target_velocity = -target_velocity
            self.set_velocity(ser, motor_id, int(target_velocity))

    def set_velocity_all(self, velocity):
        """기존 방식 유지 (직진용)"""
        for i, (ser, motor_id) in enumerate(zip(self.serial_connections, self.velocity_ids)):
            target_velocity = velocity if i % 2 != 0 else -velocity
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
        self.inwheel_ctrl = InWheelMotorController(port_list=["/dev/ttyACM1", "/dev/ttyACM2", "/dev/ttyACM3", "/dev/ttyACM4"])
        self.steering_ctrl = SteeringMotorController(port_name='/dev/ttyACM0', steering_ids=(0,1))
        self.timer = self.create_timer(0.01, self.control_loop)

    def keyboard_callback(self, msg: String):
        self.keyboard_ctrl.process_key_event(msg.data)

    def control_loop(self):
        # 아크만 기하학을 적용한 속도 및 조향 제어
        wheel_speeds, steering_angles = self.steering_ctrl.update_steering(
            self.keyboard_ctrl.steering, 
            self.keyboard_ctrl.velocity
        )
        
        # 개별 바퀴 속도 제어
        self.inwheel_ctrl.set_velocity_individual(wheel_speeds)
        
        # 상태 메시지 업데이트 (4개 바퀴 속도 포함)
        status = (f"속도: {self.keyboard_ctrl.velocity} | "
                  f"조향: {self.keyboard_ctrl.steering}° | "
                  f"FL: {wheel_speeds[0]:.1f}, FR: {wheel_speeds[1]:.1f}, "
                  f"RL: {wheel_speeds[2]:.1f}, RR: {wheel_speeds[3]:.1f} | "
                  f"조향각 - L: {steering_angles[0]:.1f}°, R: {steering_angles[1]:.1f}°")
        
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
