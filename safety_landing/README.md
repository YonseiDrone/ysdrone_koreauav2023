# safety_landing

- safety_landing package는 3개의 노드로 구성.
- [aruco_VIO](./scripts/aruco_VIO.py) + ([PID_control_node](./scripts/PID_control_node.py) or [RL_aruco_control_node](./scripts/RL_aruco_control_node.py))로 사용.
- control node에서 계산된 값은 offboard 패키지의 [control_node](././offboard/scripts/control_node.py) node로 전송.

## "aruco_VIO" node

- casade 구조인 aruco marker를 이용해 ENU 좌표계 기준 드론의 현재 위치를 파악.
- 카메라 센서의 noise를 줄이기 위해 kalman filter 사용.
- Coordinate Transformation을 간단하게 하기 위해 yaw는 90 degrees(north)로 고정하고 착륙.

## "PID_control_node" node

- aruco_VIO node로부터 얻은 위치 정보를 바탕으로 PID control.
- aruco marker가 인식되면 PID control, 인식되지 않으면 position control.

## "RL_aruco_control_node" node

- aruco_VIO node로부터 얻은 위치 정보를 바탕으로 reinforcement learning 이용해 착륙.
- 이때, Soft Actor-Critic(SAC) 알고리즘을 Unity 환경에서 학습.
- Parameter 값을 onnx 파일로 저장
- Unity와 MAVROS의 좌표계가 서로 다르기에 주의.
- PID_control_node와 마찬가지로 aruco marker가 인식되지 않으면 position control.
