import threading
import time
from dynamixel_sdk import *
from pynput import keyboard #pip3 install pynput

# ----------------------------------
# 기본 설정 및 모터 파라미터
# ----------------------------------
PROTOCOL_VERSION = 2.0
BAUDRATE = 1000000
DEVICENAME = '/dev/ttyACM0'

# 모터 ID 그룹 (속도 제어 모터와 각도 제어 모터 분리)
VELOCITY_IDS = [1, 2, 3, 4]   # 전진/후진 (속도 제어)
STEERING_IDS = [5, 6]         # 좌/우 조향 (각도 제어)

# 주소 및 모드 (속도 제어)
ADDR_TORQUE_ENABLE_VELOCITY = 64
ADDR_OPERATING_MODE_VELOCITY = 11
ADDR_GOAL_VELOCITY = 104
VELOCITY_MODE = 1  # 속도 제어 모드

# 주소 및 모드 (각도 제어)
ADDR_TORQUE_ENABLE_STEERING = 64
ADDR_OPERATING_MODE_STEERING = 11
ADDR_GOAL_POSITION = 116
ADDR_PRESENT_POSITION = 132
POSITION_MODE = 3  # 위치(각도) 제어 모드

TORQUE_ENABLE = 1
TORQUE_DISABLE = 0

# 제어 파라미터
VELOCITY_STEP = 4      # 전진 속도 증감 단위
VELOCITY_MAX = 200
VELOCITY_MIN = -200

STEERING_STEP = 10     # 조향 각 증감 단위 (°)
STEERING_MAX = 180
STEERING_MIN = 0

# ----------------------------------
# 전역 제어 변수 및 키 상태 집합
# ----------------------------------
forward_velocity = 0   # 전진/후진 속도 (양수: 전진, 음수: 후진)
steering_angle = 0     # 조향 각도 (양수: 우회전, 음수: 좌회전)
exit_flag = False      # 종료 플래그
keys_pressed = set()   # 현재 눌린 키 저장

def angle_to_position(angle):
    """0~360° 범위를 다이나믹셀의 포지션 값(0~4095)으로 변환"""
    return int((angle / 360.0) * 4095)

# ----------------------------------
# 포트 및 패킷 핸들러 초기화
# ----------------------------------
portHandler = PortHandler(DEVICENAME)
packetHandler = PacketHandler(PROTOCOL_VERSION)

if not portHandler.openPort():
    print("포트를 열 수 없습니다.")
    exit(1)

if not portHandler.setBaudRate(BAUDRATE):
    print("보드레이트 설정 실패")
    exit(1)

# ----------------------------------
# 모터 초기화
# ----------------------------------
# 속도 제어 모터 초기화
for dxl_id in VELOCITY_IDS:
    packetHandler.write1ByteTxRx(portHandler, dxl_id, ADDR_TORQUE_ENABLE_VELOCITY, TORQUE_DISABLE)
    packetHandler.write1ByteTxRx(portHandler, dxl_id, ADDR_OPERATING_MODE_VELOCITY, VELOCITY_MODE)
    packetHandler.write1ByteTxRx(portHandler, dxl_id, ADDR_TORQUE_ENABLE_VELOCITY, TORQUE_ENABLE)

# 조향(위치 제어) 모터 초기화
for dxl_id in STEERING_IDS:
    packetHandler.write1ByteTxRx(portHandler, dxl_id, ADDR_TORQUE_ENABLE_STEERING, TORQUE_DISABLE)
    packetHandler.write1ByteTxRx(portHandler, dxl_id, ADDR_OPERATING_MODE_STEERING, POSITION_MODE)
    packetHandler.write1ByteTxRx(portHandler, dxl_id, ADDR_TORQUE_ENABLE_STEERING, TORQUE_ENABLE)

# ----------------------------------
# pynput 키 이벤트 콜백 함수
# ----------------------------------
def on_press(key):
    global exit_flag
    try:
        k = key.char.lower()  # 문자 키
    except AttributeError:
        k = str(key)          # 특수 키
    keys_pressed.add(k)
    # 백스페이스 키가 눌리면 종료
    if k == 'Key.backspace':
        exit_flag = True

def on_release(key):
    try:
        k = key.char.lower()
    except AttributeError:
        k = str(key)
    if k in keys_pressed:
        keys_pressed.remove(k)

# 키 리스너 시작 (데몬 스레드)
listener = keyboard.Listener(on_press=on_press, on_release=on_release)
listener.daemon = True
listener.start()

# ----------------------------------
# 키 상태를 기반으로 제어 변수 업데이트 함수 (별도 스레드)
# ----------------------------------
def update_controls():
    global forward_velocity, steering_angle
    while not exit_flag:
        # 전진/후진 제어: w/s
        if 'w' in keys_pressed:
            forward_velocity = min(forward_velocity + VELOCITY_STEP, VELOCITY_MAX)
        if 's' in keys_pressed:
            forward_velocity = max(forward_velocity - VELOCITY_STEP, VELOCITY_MIN)
        # 조향 제어: a/d
        if 'd' in keys_pressed:
            steering_angle = min(steering_angle + STEERING_STEP, STEERING_MAX)
        if 'a' in keys_pressed:
            steering_angle = max(steering_angle - STEERING_STEP, STEERING_MIN)
        # 정지 명령: r
        if 'r' in keys_pressed:
            forward_velocity = 0
        time.sleep(0.05)

control_thread = threading.Thread(target=update_controls, daemon=True)
control_thread.start()

print("제어 시작: w/s - 전진/후진, a/d - 좌/우 회전, r - 정지, 백스페이스 - 종료")

# ----------------------------------
# 메인 제어 루프: 모터에 명령 전송
# ----------------------------------
try:
    while not exit_flag:
        # 속도 제어 모터에 전진/후진 명령 전송 (홀수/짝수 모터에 대해 부호 반전 적용)
        for i, dxl_id in enumerate(VELOCITY_IDS):
            target_velocity = forward_velocity if i % 2 == 0 else -forward_velocity
            packetHandler.write4ByteTxRx(portHandler, dxl_id, ADDR_GOAL_VELOCITY, target_velocity)
        # 조향 모터에 목표 위치(각도 변환)를 전송
        target_position = angle_to_position(steering_angle)
        for dxl_id in STEERING_IDS:
            packetHandler.write4ByteTxRx(portHandler, dxl_id, ADDR_GOAL_POSITION, target_position)
        
        # 현재 상태 출력 (한 줄로 갱신)
        print("속도: {}   조향각: {}".format(forward_velocity, steering_angle), end='\r')
        time.sleep(0.01)
except KeyboardInterrupt:
    exit_flag = True
finally:
    # 모터 정지 및 토크 비활성화 처리
    for dxl_id in VELOCITY_IDS:
        packetHandler.write4ByteTxRx(portHandler, dxl_id, ADDR_GOAL_VELOCITY, 0)
        packetHandler.write1ByteTxRx(portHandler, dxl_id, ADDR_TORQUE_ENABLE_VELOCITY, TORQUE_DISABLE)
    for dxl_id in STEERING_IDS:
        packetHandler.write4ByteTxRx(portHandler, dxl_id, ADDR_GOAL_POSITION, angle_to_position(0))
        packetHandler.write1ByteTxRx(portHandler, dxl_id, ADDR_TORQUE_ENABLE_STEERING, TORQUE_DISABLE)
    portHandler.closePort()
    print("\n프로그램 종료 완료!")
