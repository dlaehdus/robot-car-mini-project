import cv2
import numpy as np
import pyrealsense2 as rs #pip install pyrealsense2
from ultralytics import YOLO #pip install ultralytics

import msvcrt

#yolo 임포트
model = YOLO('yolov8s.pt')

#리얼센스 초기 설정 과정 (파이프라인 ,config)
pipeline = rs.pipeline() #rs 그자체로 카메라 불러옴
config = rs.config()
config.enable_stream(rs.stream.color, 640, 480, rs.format.bgr8, 30)
config.enable_stream(rs.stream.depth, 640, 480, rs.format.z16, 30)
pipeline.start(config)

#cm에서 m변환 위해서
depth_scale = 0.0010000000474974513 #depth_image * depth_scale을 통해 cm를 m로 변환

#이미지 필터
spatial = rs.spatial_filter() #이미지를 부드럽게 해주는 필터
temporal = rs.temporal_filter() #이미지를 부드럽게 해주는 필터 (spatial과는 다르게 calculating multiple frames라고 나와있음), 굳이 안해도됨

#동작 시작
try:
    while True:
        #색깔 프레임, 거리 프레임 얻어오기
        frames = pipeline.wait_for_frames() 
        color_frame = frames.get_color_frame()
        depth_frame = frames.get_depth_frame()

        #수치 행렬화
        color_image = np.asanyarray(color_frame.get_data()) #ndarray(n차원행렬)로 변환, 색깔 프레임의 데이터를 n차원행렬화
        depth_image = np.asanyarray(depth_frame.get_data()) #ndarray(n차원행렬)로 변환, 이번에 뎁스에 대한 프레임을 n차원행렬화, 이렇게 수치화를 시켜야 속도가 향상된다.

        #거리 프레임에 필터 적용
        depth_frame = spatial.process(depth_frame)
        depth_frame = temporal.process(depth_frame)
        depth_image = np.asanyarray(depth_frame.get_data())

        #cm to m 변환
        depth_image = depth_image * depth_scale #센티미터를 미터화

        #원하는 클래스만 얻어오도록 설정, yolo에 실시간 감지 트리거해주는 단계
        wanted_classes = [i for i in range(0,80) if i != 0]
        #unwanted_classes = 0
        results = model(source=color_image, classes = wanted_classes)
        #results = model(source=color_image, classes = 0)

        

        #본격적인 영상 처리 작업
        for result in results: #영상처리에서 이 문장은 필수적
            boxes = result.boxes
            for box in boxes:
                x1, y1, x2, y2 = box.xyxy[0].cpu().numpy().astype(int)
                confidence = box.conf[0].cpu().numpy()
                class_id = box.cls[0].cpu().numpy()

                if confidence < 0.5:
                    continue  # 신뢰도가 0.5보다 작으면 이 조건문에서 (계속)고립되어 있고, 0.5 이상이면 밑의 명령들을 실행한다. 

                #물체와의 거리를 계산
                object_depth = np.median(depth_image[y1:y2, x1:x2]) #np.median은 객체의 거리에 대한 중간값 
                #label = f"{object_depth:.2f}m"
                #object_name = f"{model.names[int(class_id)]}m"
                depth_object_name = f"{model.names[int(class_id)]}, {object_depth:.2f}m"

                #사각형 처리
                cv2.rectangle(color_image, (x1, y1), (x2, y2), (64, 224, 208), 1, cv2.LINE_AA)

                #텍스트 기입 (오브젝트 이름과 그 거리)
                cv2.putText(color_image, depth_object_name, (x1, y1-10), cv2.FONT_HERSHEY_DUPLEX, 0.8, (127, 255, 212), 1)
                
                key = msvcrt.getch()
                #거리가 0.8m 이하이면 아래의 명령문 수행
                if object_depth <= 0.5:
                    key in ('\x7f')

                #필요할 경우 터미널에 오브젝트 이름과 그 거리 출력
                #print(f"{model.names[int(class_id)]}: {object_depth:.2f}m")

        #이미지 디스플레잉
        cv2.imshow("Color Image", color_image)
        if cv2.waitKey(1) & 0xFF == ord('q'):
            break
#중단
finally:
    pipeline.stop()
    cv2.destroyAllWindows()