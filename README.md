# 2023 Mando_Morai 자율주행 챌린지

### 대회 소개
Morai의 디지털 트윈 기술로 구축된 가상 환경에서 자율주행 알고리즘의 검증 능력을 겨룹니다. 참가 팀들은 가상의 운전면허시험장에서 자율주행차가 실제 운전면허 시험과 동일한 미션을 수행하도록 설정해야 합니다. 수행해야 할 미션에는 차량 출발, 차로 및 지정 속도 준수, 경사로 정지 및 출발, 직각 코스 주행, 교차로 통과, S자 코스 주행, T자 주차, 고속 운전, 돌발 상황 대처 등이 포함됩니다.

<img width="370" alt="image" src="https://github.com/user-attachments/assets/97c9ffcc-664d-4606-adf9-4baa62302661">

### 기술 개요

#### Path Tracking
- 경로는 사전에 정의된 CSV 파일로 작성되며, 경로를 부드럽게 연결하기 위해 보간(interpolation) 작업이 진행됩니다.
- Pure Pursuit 알고리즘을 적용하여 자율주행차의 속도에 맞는 하이퍼 파라미터를 설정했습니다. 이를 통해 **Advanced Pure Pursuit**를 구현하여 경로 추종 성능을 향상시켰습니다.

#### GPS 음영구역 주행
- **직각 GPS 음영구역**: LiDAR를 사용하여 주변의 꼬깔콘을 인식하고 이를 기반으로 주행합니다.
- **커브 GPS 음영구역**: 카메라를 통해 차선을 인식하여 주행합니다.

#### 신호등 및 돌발 상황 대처
- 신호와 장애물 인식을 위해 Roboflow를 사용하여 데이터를 라벨링하고, Yolo v8 모델을 통해 학습시켰습니다.
- 학습된 모델을 통해 차량이 신호 및 돌발 상황에 맞춰 의사결정을 할 수 있도록 구현했습니다.
