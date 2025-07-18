import rclpy
from rclpy.node import Node
from std_msgs.msg import String
from pynput import keyboard

class RemoteControllerNode(Node):
    def __init__(self):
        super().__init__('remote_controller')
        self.publisher_ = self.create_publisher(String, 'keyboard_input', 10)
        self.subscription = self.create_subscription(String, 'motor_status', self.motor_status_callback, 10)
        self.listener = keyboard.Listener(on_press=self.on_press, on_release=self.on_release)
        self.listener.start()
        self.get_logger().info("원격 키보드 제어 노드 시작. 키 입력을 보내고 모터 상태를 수신합니다.")

    def on_press(self, key):
        try:
            key_char = key.char.lower()
        except AttributeError:
            key_char = str(key)
        msg = String()
        msg.data = f"press:{key_char}"
        self.publisher_.publish(msg)

    def on_release(self, key):
        try:
            key_char = key.char.lower()
        except AttributeError:
            key_char = str(key)
        msg = String()
        msg.data = f"release:{key_char}"
        self.publisher_.publish(msg)
        if key == keyboard.Key.backspace:
            self.get_logger().info("백스페이스 입력 감지 - 노드 종료.")
            self.listener.stop()
            rclpy.shutdown()

    def motor_status_callback(self, msg: String):
        self.get_logger().info("모터 상태: " + msg.data)

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
