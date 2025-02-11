import os
import sys
import time
import select

if os.name == 'nt':
    import msvcrt
    def getch():
        return msvcrt.getch().decode()
else:
    import tty
    import termios
    fd = sys.stdin.fileno()
    old_settings = termios.tcgetattr(fd)
    def getch():
        try:
            tty.setraw(sys.stdin.fileno())
            ch = sys.stdin.read(1)
        finally:
            termios.tcsetattr(fd, termios.TCSADRAIN, old_settings)
        return ch    

from dynamixel_sdk import *

# 다이나믹셀 기본 설정
PROTOCOL_VERSION = 2.0
BAUDRATE = 1000000
DEVICENAME = '/dev/ttyACM0'
DXL_IDS = [5, 6]  # 5번, 6번 다이나믹셀 ID

# 다이나믹셀 주소값
ADDR_TORQUE_ENABLE = 64
ADDR_OPERATING_MODE = 11  # 동작 모드
ADDR_GOAL_POSITION = 116  # 목표 위치 (4Byte)
ADDR_PRESENT_POSITION = 132  # 현재 위치 (4Byte)

TORQUE_ENABLE = 1
TORQUE_DISABLE = 0
POSITION_MODE = 3  # 각도 모드

# 포트 및 패킷 핸들러 초기화
portHandler = PortHandler(DEVICENAME)
packetHandler = PacketHandler(PROTOCOL_VERSION)

# 포트 열기
if not portHandler.openPort():
    print("Failed to open the port")
    exit()

# 보드레이트 설정
if not portHandler.setBaudRate(BAUDRATE):
    print("Failed to change the baudrate")
    exit()

# 다이나믹셀 설정 (각도 모드 + 토크 활성화)
for dxl_id in DXL_IDS:
    packetHandler.write1ByteTxRx(portHandler, dxl_id, ADDR_TORQUE_ENABLE, TORQUE_DISABLE)
    packetHandler.write1ByteTxRx(portHandler, dxl_id, ADDR_OPERATING_MODE, POSITION_MODE)
    packetHandler.write1ByteTxRx(portHandler, dxl_id, ADDR_TORQUE_ENABLE, TORQUE_ENABLE)

# 현재 위치 읽기 함수
def get_current_position(dxl_id):
    pos, dxl_comm_result, dxl_error = packetHandler.read4ByteTxRx(portHandler, dxl_id, ADDR_PRESENT_POSITION)
    if dxl_comm_result != COMM_SUCCESS:
        print(f"통신 실패: {packetHandler.getTxRxResult(dxl_comm_result)}")
    elif dxl_error:
        print(f"오류 발생: {packetHandler.getRxPacketError(dxl_error)}")
    return pos

# 각도를 다이나믹셀 값(0~4095)으로 변환
def angle_to_position(angle):
    return int((angle / 360.0) * 4095)

# 다이나믹셀 값을 각도로 변환
def position_to_angle(position):
    return (position / 4095.0) * 360.0

try:
    while True:
        key = getch()

        if key == '\x7f':  # Backspace (ASCII 코드 127)
            print("프로그램 종료")
            break

        for dxl_id in DXL_IDS:
            current_pos = get_current_position(dxl_id)
            current_angle = position_to_angle(current_pos)

            if key == 'd':  # +1도 회전 (최대 90도까지)
                target_angle = min(180, current_angle + 10)
            elif key == 'a':  # -1도 회전 (최소 -90도까지)
                target_angle = max(-180, current_angle - 10)
            else:
                continue

            target_pos = angle_to_position(target_angle)
            packetHandler.write4ByteTxRx(portHandler, dxl_id, ADDR_GOAL_POSITION, target_pos)
            print(f"모터 {dxl_id}: {current_angle:.1f}° → {target_angle:.1f}°")

except KeyboardInterrupt:
    print("\n프로그램 종료!")

# 모터 정지 후 종료
for dxl_id in DXL_IDS:
    packetHandler.write1ByteTxRx(portHandler, dxl_id, ADDR_TORQUE_ENABLE, TORQUE_DISABLE)

portHandler.closePort()