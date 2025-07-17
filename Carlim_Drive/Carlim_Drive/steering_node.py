import math
import sys
import rclpy
from rclpy.node import Node
from std_msgs.msg import Float64MultiArray, String  # Float64MultiArray로 변경, Twist 제거
from dynamixel_sdk import PortHandler, PacketHandler

class SteeringMotorController:
    def __init__(self, port_name='/dev/ttyACM0', baud_rate=1000000, steering_ids=(0, 1)):
        self.port = PortHandler(port_name)
        self.packet = PacketHandler(2.0)
        if not (self.port.openPort() and self.port.setBaudRate(baud_rate)):
            print("포트를 열거나 보드레이트 설정에 실패했습니다.")
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

        self.current_positions = [self.INIT_POSITION for _ in self.steering_ids]
        self.STEERING_SMOOTH_STEP = 5  

    def steering_to_position(self, angle):
        return self.INIT_POSITION + int((angle / 45.0) * 512)

    def apply_steering_angles(self, angles):  # 새 메서드: 받은 각도 배열을 부드럽게 적용 (update_steering 대체)
        if len(angles) != 2:
            print("[ERROR] Invalid angles length, using default [0.0, 0.0]")
            angles = [0.0, 0.0]
        
        target_angles = angles + [0.0, 0.0]  # 뒷바퀴 고정: [FL, FR, RL=0, RR=0]으로 확장 (4WS 확장 대비)
        
        target_positions = [self.steering_to_position(a) for a in target_angles[:2]]  # 앞바퀴 2개만 (뒷바퀴 무시 가정)
        diffs = [target_positions[i] - self.current_positions[i] for i in range(len(target_positions))]
        abs_diffs = [abs(d) for d in diffs]
        max_diff = max(abs_diffs) if abs_diffs else 0
        
        if max_diff == 0:
            for i, motor_id in enumerate(self.steering_ids):
                self.packet.write4ByteTxRx(self.port, motor_id, 116, int(self.current_positions[i]))
            return target_angles[:2]  # 앞 2개만 반환
        
        for i, motor_id in enumerate(self.steering_ids):
            diff = diffs[i]
            if abs(diff) < 1:
                new_position = target_positions[i]
            else:
                step = self.STEERING_SMOOTH_STEP * abs(diff) / max_diff
                new_position = self.current_positions[i] + (step if diff > 0 else -step)
            self.current_positions[i] = new_position
            self.packet.write4ByteTxRx(self.port, motor_id, 116, int(new_position))
        return target_angles[:2]  # 앞 2개만 반환

    def shutdown(self):
        for _id in self.steering_ids:
            self.packet.write4ByteTxRx(self.port, _id, 116, self.INIT_POSITION)
            self.packet.write1ByteTxRx(self.port, _id, 64, 0)
        self.port.closePort()


class SteeringNode(Node):
    def __init__(self):
        super().__init__('steering_node')
        self.subscription = self.create_subscription(Float64MultiArray, '/steering_angles', self.angle_callback, 10)  # 수정: /steering_angles, Float64MultiArray
        self.publisher = self.create_publisher(String, '/motor_status', 10)  # 상태 피드백
        self.steering_ctrl = SteeringMotorController()
        self.angles = [0.0, 0.0]  # 초기 각도 배열
        self.timer = self.create_timer(0.01, self.control_loop)

    def angle_callback(self, msg: Float64MultiArray):
        self.angles = msg.data  # [left, right] 추출
        if len(self.angles) != 2:
            print("[ERROR] Invalid angles length, resetting to [0.0, 0.0]")
            self.angles = [0.0, 0.0]

    def control_loop(self):
        applied_angles = self.steering_ctrl.apply_steering_angles(self.angles)  # 수정: 받은 각도 직접 적용
        status = f"조향: Left:{applied_angles[0]:.1f}°, Right:{applied_angles[1]:.1f}°"
        self.publisher.publish(String(data=status))
        sys.stdout.write('\r' + status)
        sys.stdout.flush()

    def destroy_node(self):
        self.steering_ctrl.shutdown()
        super().destroy_node()

def main():
    rclpy.init()
    node = SteeringNode()
    try:
        rclpy.spin(node)
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == "__main__":
    main()
