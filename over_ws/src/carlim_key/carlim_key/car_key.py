import rclpy                                    # 노드 생성, 메시지 전송/수신 등의 기본 기능을 제공
from rclpy.node import Node                     # 모든 ROS2 노드는 이 클래스를 상속
from std_msgs.msg import String                 # 문자열 메시지 타입을 임포트합니다. 키보드 입력 데이터를 문자열로 전송하기 위해 사용
from pynput import keyboard                     # 키보드 입력을 감지하고 처리하는 라이브러리를 임포트

'''
키 입력을 처리하고 해당 키 값을 발행하는 클래스
'''
class RemoteControllerNode(Node):
    # 초기화
    def __init__(self):
        super().__init__('remote_controller')                                                                   # 부모 클래스(Node)의 생성자를 호출하여 노드 이름을 'remote_controller'로 설정합니다.
        
        # 퍼블리셔와 서브스크라이버 생성
        self.publisher_ = self.create_publisher(String, 'keyboard_input', 10)                                   # 키보드 입력을 전송하는 퍼블리셔를 생성, 메시지 타입: String, 토픽명: 'keyboard_input', 큐 크기: 10
        self.subscription = self.create_subscription(String, 'motor_status', self.motor_status_callback, 10)    # 모터 상태를 수신하는 서브스크라이버를 생성, 메시지 타입: String, 토픽명: 'motor_status', 콜백 함수: motor_status_callback, 큐 크기: 10
        
        # 키보드 받아들이는거 설정
        self.listener = keyboard.Listener(on_press=self.on_press, on_release=self.on_release)                   # 키보드 이벤트를 감지하는 리스너를 생성, 키가 눌렸을 때: on_press 메서드 호출, 키가 떼졌을 때: on_release 메서드 호출
        self.listener.start()                                                                                   # 키보드 리스너를 시작합니다. 이제 키보드 입력을 감지할 수 있습니다.
        self.get_logger().info("[INFO] 원격 키보드 제어 노드 시작. 키 입력을 보내고 모터 상태를 수신합니다.")                # 노드 시작 메시지를 로그에 출력합니다.

    # 키 눌리는걸 처리함
    def on_press(self, key):                # 키가 눌렸을 때 호출되는 메서드
        try:
            key_char = key.char.lower()     # 일반 문자 키(a-z, 0-9 등)의 경우 소문자로 변환
        except AttributeError:           
            key_char = str(key)             # 특수 키(화살표, Ctrl, Alt 등)의 경우 문자열로 변환
        msg = String()                      # 새로운 String 메시지 객체를 생성
        msg.data = f"press:{key_char}"      # 메시지 데이터를 "press:키이름" 형태로 설정
        self.publisher_.publish(msg)        # 메시지를 'keyboard_input' 토픽으로 전송

    # 키 떨어짐을 처리함
    def on_release(self, key):              # 키가 떼졌을 때 호출되는 메서드
        try:
            key_char = key.char.lower()
        except AttributeError:
            key_char = str(key)
        msg = String()
        msg.data = f"release:{key_char}"
        self.publisher_.publish(msg)
        if key == keyboard.Key.backspace:                                   # 백스페이스 키가 떼졌는지 확인
            self.get_logger().info("[INFO] 백스페이스 입력 감지 - 노드 종료.")           # 종료 메시지를 로그에 출력    
            self.listener.stop()                                            # 키보드 리스너를 중지
            rclpy.shutdown()                                                # ROS2 시스템을 종료

    def motor_status_callback(self, msg: String):               # 모터 상태 메시지를 수신할 때 호출되는 콜백 함수
        self.get_logger().info("[INFO] 모터 상태: " + msg.data)          # 수신된 모터 상태를 로그에 출력

def main():
    rclpy.init()                    # ROS2 시스템을 초기화
    node = RemoteControllerNode()   # RemoteControllerNode 인스턴스를 생성
    try:
        rclpy.spin(node)            # 노드를 실행
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == "__main__":
    main()
