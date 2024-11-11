# 2023 Mando_Morai 자율주행 챌린지
### 대회 소개 
Morai의 디지털 트윈 기술로 구축된 가상의 환경에서 자신의 자율주행 알고리즘의 검증 능력을 겨룸.
참가 팀들은 가상의 운전면허시험장에서 차량 출발에서부터, 차로 및 지정속도 준수, 경사로 정지, 출발, 직각 코스, 교차로,
S자 코스, T자 주차, 고속 운전, 돌발상황 등 운전자들이 실제 운전면허 시험에서 수행하는 동일 미션을 자율주행차량이 수행하도록 함.
<img width="370" alt="image" src="https://github.com/user-attachments/assets/97c9ffcc-664d-4606-adf9-4baa62302661">

#### Path Tracking
정해진 경로를 csv파일로 만들어 interpolation을 함
Pure Pursuit 알고리즘을 사용했으며, 자차의 속도에 맞는 pure pursuit 하이퍼 파라미터를 적용하여 
Advanced pure pursuit 적용

#### GPS 음영구역
[직각 GPS 음영구역]
LiDAR로 꼬깔콘을 인식하여 주행
[커브 GPS 음영구역]
Camera를 통한 차선인식을 통해 주행

#### 신호등 및 돌발 상황 대처
Roboflow로 신호 및 장애물을 labeling 작업을 거친 후, Yolo v8모델로 학습함
학습 모델을 통해, 차량의 decision making알고리즘을 수행

