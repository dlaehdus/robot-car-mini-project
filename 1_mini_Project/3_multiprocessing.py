import os
import sys
import time
import threading
import select

# 플랫폼에 따라 비차단 키 입력 함수 정의 (Windows 및 Unix 호환)
if os.name == 'nt':
    import msvcrt
    def getch_nonblocking():
        if msvcrt.kbhit():
            return msvcrt.getch().decode()
        return None
else:
    import tty, termios
    fd = sys.stdin.fileno()
    old_settings = termios.tcgetattr(fd)
    tty.setcbreak(fd)
    def getch_nonblocking():
        dr, dw, de = select.select([sys.stdin], [], [], 0)
        if dr:
            return sys.stdin.read(1)
        return None

from dynamixel_sdk import *

# ----------------------------------
# 기본 설정 및 모터 파라미터
# ----------------------------------
PROTOCOL_VERSION = 2.0
BAUDRATE = 1000000
DEVICENAME = '/dev/ttyACM0'

# 모터 ID 그룹 (속도 제어 모터와 각도 제어 모터 분리)
VELOCITY_IDS = [1, 2, 3, 4]  # 전진/후진 (속도 제어)
STEERING_IDS = [5, 6]        # 좌/우 조향 (각도 제어)

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
# 전역 제어 변수 (키 입력에 의해 업데이트됨)
# ----------------------------------
forward_velocity = 0   # 전진/후진 속도 (양수: 전진, 음수: 후진)
steering_angle = 0       # 조향 각도 (양수: 우회전, 음수: 좌회전)
exit_flag = False        # 종료 플래그

# ----------------------------------
# 포트 및 패킷 핸들러 초기화
# ----------------------------------
portHandler = PortHandler(DEVICENAME)
packetHandler = PacketHandler(PROTOCOL_VERSION)

if not portHandler.openPort():
    print("포트를 열 수 없습니다.")
    sys.exit(1)

if not portHandler.setBaudRate(BAUDRATE):
    print("보드레이트 설정 실패")
    sys.exit(1)

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

def angle_to_position(angle):
    """0~360° 범위를 다이나믹셀의 포지션 값(0~4095)으로 변환"""
    return int((angle / 360.0) * 4095)

# ----------------------------------
# 키 입력 처리 스레드
# ----------------------------------
def key_listener():
    global forward_velocity, steering_angle, exit_flag
    while not exit_flag:
        key = getch_nonblocking()
        if key is not None:
            # 백스페이스 (Windows: '\x08', Unix: '\x7f') 입력 시 종료
            if key in ('\x08', '\x7f'):
                exit_flag = True
            # 전진/후진 속도 제어
            if key == 'w':
                forward_velocity = min(forward_velocity + VELOCITY_STEP, VELOCITY_MAX)
            elif key == 's':
                forward_velocity = max(forward_velocity - VELOCITY_STEP, VELOCITY_MIN)
            # 조향(회전) 제어
            if key == 'd':
                steering_angle = min(steering_angle + STEERING_STEP, STEERING_MAX)
            elif key == 'a':
                steering_angle = max(steering_angle - STEERING_STEP, STEERING_MIN)
            # 정지 명령
            if key == 'r':
                forward_velocity = 0
        time.sleep(0.01)

# 키 입력 처리 스레드를 데몬 스레드로 실행
listener_thread = threading.Thread(target=key_listener, daemon=True)
listener_thread.start()

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
    if os.name != 'nt':
        termios.tcsetattr(fd, termios.TCSADRAIN, old_settings)
    print("\n프로그램 종료 완료!")
