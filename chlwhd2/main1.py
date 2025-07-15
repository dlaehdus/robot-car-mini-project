import rclpy
from rclpy.node import Node
from std_msgs.msg import String
from pynput import keyboard
import sys

class RemoteControllerNode(Node):
    def __init__(self):
        super().__init__('remote_controller')
        self.publisher_ = self.create_publisher(String, 'keyboard_input', 10)
        self.subscription = self.create_subscription(String, 'motor_status', self.motor_status_callback, 10)
        self.listener = keyboard.Listener(on_press=self.on_press, on_release=self.on_release)
        self.listener.start()
        # 초기 메시지는 한 번만 출력 (줄바꿈 포함)
        sys.stdout.write("원격 키보드 제어 노드 시작. 키 입력을 보내고 모터 상태를 수신합니다.\n")
        sys.stdout.flush()

    def on_press(self, key):
        try:
            key_char = key.char.lower()
        except AttributeError:
            key_char = str(key)
        msg = String()
        msg.data = f"press:{key_char}"
        self.publisher_.publish(msg)
        # on_press에서는 별도의 출력 없음

    def on_release(self, key):
        try:
            key_char = key.char.lower()
        except AttributeError:
            key_char = str(key)
        msg = String()
        msg.data = f"release:{key_char}"
        self.publisher_.publish(msg)
        if key == keyboard.Key.backspace:
            # 백스페이스 입력 시 고정된 줄 출력에 줄바꿈 추가 후 종료
            sys.stdout.write("\n백스페이스 입력 감지 - 노드 종료.\n")
            sys.stdout.flush()
            self.listener.stop()
            rclpy.shutdown()

    def motor_status_callback(self, msg: String):
        # ANSI 이스케이프 시퀀스로 현재 줄 클리어 후 출력(줄 고정)
        line = "모터 상태: " + msg.data
        sys.stdout.write("\r\033[K" + line)
        sys.stdout.flush()

def main():
    rclpy.init()
    node = RemoteControllerNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == "__main__":
    main()
