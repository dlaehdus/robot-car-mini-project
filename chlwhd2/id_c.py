from id import MotorController  
# MotorController 클래스를 file1에서 임포트

controller = MotorController(port='/dev/ttyACM2')

# controller.id_set(0x02)
controller.id_query()

# controller.query_velocity_and_angle(0x02)

# controller.switch_current_mode(0x02)
# controller.switch_velocity_mode(0x02)
# controller.switch_angle_mode(0x02)

# controller.set_velocity(0x02, 0)
# controller.set_angle(0x02, 180)
# controller.set_relative_angle(0x02, 35)

# controller.brake(0x00)