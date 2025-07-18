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
        # 현재 눌려있는 키를 저장함. 파이썬 내장 데이터 타입임, 별도의 모듈 없이 사용
        # 중복을 허용하지 않는 데이터 구조, 순서가 없는 데이터 구조
        self.velocity = 0
        self.steering = 0

        self.robot_exit = False
        # 백스페이스 입력시 True로 변하여 종료됌

        self.VELOCITY_STEP = 10
        self.STEERING_STEP = 5

        self.VELOCITY_MAX = 100
        self.VELOCITY_MIN = -100

        self.STEERING_MAX = 60
        self.STEERING_MIN = -60

        self.RETURN_VELOCITY = 5
        self.RETURN_STEERING = 10

    def process_key_event(self, event: str):
    # event라는 문자열을 입력받아, 키 입력 이벤트를 처리하는 함수입니다.
    # 예를 들어 event = "press:w" 또는 event = "release:a" 같은 문자열이 들어옵니다.
        if event.startswith("press:"):
        # startswith()는 문자열이 특정 문자(또는 문자열)로 시작하는지 확인하는 함수야. True 또는 False를 반환해.
        # event.startswith("press:"): event가 "press:"로 시작하면 키가 눌렸다는 뜻.
            key = event[len("press:"):].strip().lower()
            # "press:w"에서 "press:" 부분을 제거하면 "w"가 남음.
            # .strip().lower()를 사용하여 공백 제거 후 소문자로 변환.
            if key == "backspace":
            # "press:backspace"가 들어오면 robot_exit = True로 설정하여 프로그램 종료를 유도.
                self.robot_exit = True
            else:
                self.keys_pressed.add(key)
                # 예를 들어 "press:w"가 들어오면 self.keys_pressed.add("w")를 실행하여 'w'를 집합에 추가.
        elif event.startswith("release:"):
        # "release:w"가 들어오면 "release:" 부분을 제거하고 "w"를 얻음.
            key = event[len("release:"):].strip().lower()
            self.keys_pressed.discard(key)
            # discard()는 해당 키가 집합에 없어도 에러 없이 실행됨.

    def start(self):
    # start() 함수는 KeyboardController 클래스에서 키 입력을 처리하는 스레드를 시작하는 역할.
        self.control_thread = threading.Thread(target=self.update_controls, daemon=True)
        # threading.Thread(...) : 새로운 스레드를 생성.
        # target=self.update_controls : 실행할 함수(update_controls())를 지정.
        # daemon=True : 데몬 스레드로 설정 → 메인 프로그램이 종료되면 스레드도 자동 종료됨.
        self.control_thread.start()
        # start()를 호출하면 스레드가 실행됨 → 즉, update_controls()가 백그라운드에서 계속 실행됨.

    def update_controls(self):
        while not self.robot_exit:
        # self.robot_exit가 True가 될 때까지 계속 실행됨.
        # 즉, 프로그램이 종료될 때까지 키 입력을 확인하면서 속도를 조절하는 역할.
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
        self.port = PortHandler(port_name)
        self.packet = PacketHandler(2.0)
        if not (self.port.openPort() and self.port.setBaudRate(baud_rate)):
            print("❌ 포트를 열거나 보드레이트 설정에 실패했습니다.")
            raise Exception("포트 연결 실패")
            # 예외처리
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
        # 차의 길이의 절반으로 설정
        self.rear_wheelbase = self.ROBOT_LENGTH / 2

    def steering_to_position(self, angle):
        # 45°당 512 단위로 변환 (다이나믹셀의 중립값 2047에서 오프셋)
        return self.INIT_POSITION + int((angle / 45.0) * 512)
        # 5씩 증감하고 10으로 되돌아오므로 45면 괜찮음 19단위로 변환해도 좋음

    def update_steering(self, steering_angle):
        if steering_angle == 0:
            angles = [0, 0, 0, 0]
            # 모든 바퀴의 각도를 0도로 설정합니다. 순서: [front_right, front_left, rear_right, rear_left].
        else:
            direction = 1 if steering_angle > 0 else -1
            # 스티어링 방향을 결정합니다. 양수(오른쪽 회전)면 1, 음수(왼쪽 회전)면 -1.
            input_angle = abs(steering_angle)
            # 스티어링 각도의 절대값을 사용하여 방향을 제외한 크기만 계산에 사용합니다.
            inner_angle_rad = math.radians(input_angle)
            # 내측 바퀴의 각도를 라디안으로 변환합니다.
            R = self.front_wheelbase / math.tan(inner_angle_rad) + (self.ROBOT_WIDTH / 2)
            # 회전 반경(R)을 계산합니다. 이는 아크만 스티어링에서 차량의 회전 중심까지의 거리입니다.
            outer_front_angle = math.degrees(math.atan(self.front_wheelbase / (R + self.ROBOT_WIDTH / 2)))
            # 외측 앞바퀴의 각도를 계산하고 라디안에서 도 단위로 변환합니다.
            inner_rear_angle = math.degrees(math.atan(self.rear_wheelbase / (R - self.ROBOT_WIDTH / 2)))
            outer_rear_angle = math.degrees(math.atan(self.rear_wheelbase / (R + self.ROBOT_WIDTH / 2)))
            if direction > 0:
            # 오른쪽 회전(direction = 1)인 경우입니다.
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
        # angles를 각도로 변환함
        for motor_id, pos in zip(self.steering_ids, target_positions):
        # 각 모터 ID와 대응하는 위치 값을 쌍으로 묶어 반복합니다.
            self.packet.write4ByteTxRx(self.port, motor_id, 116, pos)
        return angles

    def shutdown(self):
    # 클래스를 종료하며 모터를 초기화하고 포트를 닫는 함수입니다.
        for _id in self.steering_ids:
            self.packet.write4ByteTxRx(self.port, _id, 116, self.INIT_POSITION)
            self.packet.write1ByteTxRx(self.port, _id, 64, 0)
        self.port.closePort()

class InWheelMotorController:
    def __init__(self, port_list=("/dev/ttyACM0", "/dev/ttyACM1", "/dev/ttyACM2", "/dev/ttyACM3"),
                 velocity_ids=(0x00, 0x01, 0x02, 0x03)):
    #설명: 클래스의 생성자 메서드로, 객체가 생성될 때 호출됩니다. port_list는 시리얼 포트 목록(기본값: "/dev/ttyACM0" ~ "/dev/ttyACM3"), velocity_ids는 각 모터의 ID(기본값: 0x00 ~ 0x03)를 매개변수로 받습니다.
        self.port_list = port_list
        # 매개변수로 받은 port_list를 클래스 인스턴스의 속성으로 저장합니다. 이는 모터와 통신할 시리얼 포트 목록을 나타냅니다.
        self.velocity_ids = velocity_ids
        # 매개변수로 받은 velocity_ids를 클래스 인스턴스의 속성으로 저장합니다. 이는 각 모터에 할당된 고유 ID를 나타냅니다.
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
        # int(decimal): 입력된 속도 값을 정수로 변환합니다.
        # struct.pack('>h', ...): 정수를 2바이트 signed short로 패킹합니다. >는 빅엔디언을 의미합니다.
        # struct.unpack('>H', ...)[0]: 패킹된 값을 unsigned short로 언패킹하여 16진수로 변환 준비합니다.
        # format(..., 'x'): 값을 소문자 16진수 문자열로 변환합니다.
        # .zfill(4): 문자열을 4자리로 맞추기 위해 왼쪽에 0을 채웁니다.

        # @staticmethod는 파이썬에서 클래스의 메서드를 정의할 때 사용하는 데코레이터인데, 이걸 왜 쓰는지 궁금하신 거죠? 간단히 말해서, @staticmethod는 클래스의 인스턴스 없이도 호출할 수 있는 메서드를 만들 때 사용됩니다. 즉, 메서드가 클래스의 특정 인스턴스 상태나 속성에 의존하지 않고, 독립적으로 동작할 수 있게 해줍니다. 그럼 자세히 설명해볼게요!
        
        # 왜 @staticmethod를 쓰나요?
        # @staticmethod를 사용하는 이유는 크게 네 가지로 정리할 수 있습니다:
        
        # 인스턴스와 독립적이다
        # @staticmethod로 정의된 메서드는 self 같은 인스턴스 변수를 사용하지 않습니다.
        # 이 메서드는 오직 입력값(매개변수)에만 의존해서 동작하며, 클래스의 인스턴스 속성이나 상태와는 상관없습니다.
        # 예를 들어, 숫자를 16진수로 바꾸거나 CRC를 계산하는 작업은 특정 인스턴스와 관계없이 독립적으로 수행할 수 있죠.
        # 유틸리티 함수로 사용하기 좋다
        # 클래스 안에 있지만 인스턴스와 관련 없는 도구(유틸리티) 함수를 정의할 때 딱입니다.
        # 이런 함수들은 보통 데이터 변환, 계산, 전송 같은 작업을 맡는데, 특정 객체의 상태를 건드릴 필요가 없어요.
        # 코드를 깔끔하게 정리할 수 있다
        # 관련된 기능을 한 클래스 안에 묶어서 관리할 수 있어요.
        # 예를 들어, 모터 제어와 관련된 유틸리티 함수들을 InWheelMotorController라는 클래스 안에 넣으면 코드가 더 체계적으로 보이죠.
        # 인스턴스 만들지 않고 바로 호출 가능
        # @staticmethod가 붙은 메서드는 클래스 이름으로 바로 호출할 수 있습니다.
        # 예: InWheelMotorController.calculate_crc()
        # 인스턴스를 만들지 않아도 되니까 메모리도 아끼고, 불필요한 작업을 줄일 수 있어요.
        # 실제 코드에서 어떻게 쓰이나요?
        # 예를 들어, 이런 메서드들이 @staticmethod로 정의될 수 있어요:
        
        # decimal_to_hex_bytes: 정수를 16진수 바이트로 변환하는 함수.
        # calculate_crc: 데이터의 CRC(오류 검출 코드)를 계산하는 함수.
        # send_data: 시리얼 포트를 통해 데이터를 보내는 함수.
        # 이 메서드들은 모두 특정 인스턴스의 데이터에 의존하지 않고 입력값만 있으면 독립적으로 작동합니다. 그러니까 @staticmethod를 써서 클래스 안에 넣어두고, 필요할 때마다 인스턴스 없이 호출할 수 있게 한 거예요.
        
        # 인스턴스(instance)는 클래스로부터 생성된 실제 객체를 의미해요. 쉽게 말해, 클래스는 객체를 만들기 위한 설계도 같은 개념이고, 그 설계도를 바탕으로 실제로 만들어진 것이 인스턴스죠. 예를 들어, "자동차"라는 클래스가 있다면, 그 클래스를 통해 만들어진 "내 자동차"나 "친구의 자동차"가 각각 인스턴스가 되는 겁니다.
        
        # 클래스와 인스턴스의 관계
        # 클래스(Class): 객체를 만들기 위한 틀이에요. 속성(데이터)과 메서드(기능)를 미리 정의해둔 설계도라고 할 수 있죠.
        # 인스턴스(Instance): 클래스를 기반으로 메모리에 실제로 생성된 객체예요. 설계도를 따라 만들어진 실체인 셈이죠.
        # 예를 들어, "사람"이라는 클래스가 있다면, "홍길동"이나 "김철수"는 그 클래스의 인스턴스예요. 각각의 인스턴스는 독립적인 속성(이름, 나이 등)을 가질 수 있고, 클래스에 정의된 행동(예: 걷기, 말하기)을 수행할 수 있어요.
        
        # 인스턴스의 특징
        # 독립성: 각 인스턴스는 서로 독립적이에요. 한 인스턴스의 속성을 바꿔도 다른 인스턴스에는 영향을 주지 않죠.
        # 속성과 메서드: 클래스가 정의한 속성과 메서드를 그대로 가져와요. 예를 들어, "자동차" 클래스에 "색상" 속성과 "주행" 메서드가 있다면, 모든 인스턴스는 색상을 가지고 주행할 수 있어요.
        # 메모리 할당: 인스턴스가 생성될 때마다 메모리에 새로운 공간이 할당돼요. 그래서 각 인스턴스는 고유한 데이터를 가질 수 있죠.

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
    # 특정 모터에 속도 명령을 보내는 메서드입니다. ser는 시리얼 연결, ID는 모터 ID, speed는 설정할 속도입니다.
        speed_H, speed_L = self.decimal_to_hex_bytes(speed)
        # decimal_to_hex_bytes 메서드를 호출하여 속도를 상위 바이트(speed_H)와 하위 바이트(speed_L)로 변환합니다.
        data_temp = bytes([int(ID) & 0xFF,0x64,int(speed_H) & 0xFF,int(speed_L) & 0xFF,0, 0, 0, 0, 0])
        # 설명: 전송할 데이터를 구성합니다.
        # 1바이트: 모터 ID (8비트로 마스킹)
        # 2바이트: 명령 코드 0x64 (속도 설정 명령으로 추정)
        # 3-4바이트: 속도의 상위 및 하위 바이트
        # 5-9바이트: 0 (패딩 또는 예약된 필드)
        crc = self.calculate_crc(data_temp)
        data = data_temp + bytes([crc])
        self.send_data(ser, data)

    def brake(self, ser, ID):
        data_temp = bytes([int(ID) & 0xFF,0x64,0, 0, 0, 0, 0,0xFF, 0])
        crc = self.calculate_crc(data_temp)
        data = data_temp + bytes([crc])
        self.send_data(ser, data)

    def set_velocity_all(self, velocity):
    # 모든 모터에 대해 속도를 설정하는 메서드입니다. velocity는 설정할 속도 값입니다.
        for i, (ser, motor_id) in enumerate(zip(self.serial_connections, self.velocity_ids)):
        # serial_connections와 velocity_ids를 쌍으로 묶고, 인덱스(i)와 함께 반복합니다.
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
