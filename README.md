# 2023 koreauav Khadas VIM4 main-repo
**Author: Jusuk Lee, ChanJoon Park, jellyho(Hokyun Im), Inha Baek**

## offboard

Offboard 모드에서 드론을 제어하기 위한 패키지로 Service를 통한 각 미션 수행, 경로점 업데이트, 기체 비행모드 전환 등을 수행한다.
### Files
- [launch](offboard/launch)
  - [avoidance.launch](offboard/launch/avoidance.launch): Pixhawk FC와 Companion PC를 연결하고, RealSense455 카메라 구동, `control_node`를 함께 실행한다. 비행 전 구동되는 런치파일 중 하나.

  - [waypoint_server.launch](offboard/launch/waypoint_server.launch): 주어진 GPS값을 LOCAL_ENU 기준 좌표계로 변환해주고 `path_node`를 통해 기체가 해당 좌표점에 도달하였을 때 다음 경로점으로 목표점을 publish해준다. 플래너로 PX4-Avoidance를 사용하므로 `input/goal_position` 토픽의 메세지인 `MarkerArray`타입으로 발행한다.

  > [!IMPORTANT]\
  > 이 런치파일을 구동하였을 때는 경로점이 맞게 생성되었는지 반드시 확인해야 한다.
  > `path_node`에서 ros param으로 설정해주는 `destination_1_pose_x`, `destination_1_pose_y` ~ `destination_3_pose_x`, `destination_3_pose_y`와 `destination_z` 을 `rosparam get`으로 비행 전 체크해주어야 한다.
  > bash view_destination.sh 로 한번에 바로 확인 가능하다.

  - [mission.launch](offboard/launch/mission.launch): WPT#3 도달 후 수행하는 임무들(건물 탐색, 마커 인식, 화물 배송, 정밀 착륙)이 포함된 런치파일. `setmode_node`가 포함되어 있어 **앞선 런치파일들이 실행된 상태에서 런치되면 OFFBOARD 모드로 전환되므로 주의해야 한다.**
- [scripts](offboard/scripts)
  - [control_node.py](offboard/scripts/control_node.py): Node for controlling the drone.
  - [path_node.py](offboard/scripts/path_node.py): Node for planning and following a path.
  - [setmode_node.py](offboard/scripts/setmode_node.py): Node for setting the flight mode of the drone.

**TODO**

- [x] GPS 좌표계 yaml 파일을 읽어서 업데이트 해줘야 하는 경로에 추가
- [x] cpp 버전으로 작성

## Building Search

### Files

- [src](building_search/src/)
  - [building_control.cpp](building_search/src/building_search.cpp): BuildingSearch Class implementation. Check out dtailed description in [README](building_search/README.md)
  - [building_control_node.cpp](building_search/src/building_search_node.cpp): executable node which named building_search_node.

**TODO**
- [ ] 실제에서 테스트 및 주변 오브젝트 처리
- [ ] 카메라 FOV가 안나오는 경우 디버깅(정밀도 향상)

## YOLO_cross_marker_detection

Requirements:
PX4-Avoidance 패키지 안에 (PX4-Avoidance/local_planner/src/nodes/local_planner_nodelet.cpp)의 90번째 줄의 토픽 이름 변경(/mavros/setpoint_position/local -> /avoidance/setpoint_position/local)

### Files
- [src](yolo_cross_detection/scripts/)
  - [yolo_tf_node.py](yolo_cross_detection/scripts/yolo_tf_node.py):
    1) building search 패키지에서 받은 centeroid를 기준으로 원주비행
    2) 십자가 마커 인식 시 정지 후 setpoint 계산
       2-1) Yolo를 통해 십자가 인식 후 픽셀 좌표계 기준 bounding box 값들을 받아옴
       2-2) bounding box 내에서 픽셀들을 sampling
       2-3) Coordinate Transformation(pixel coordinate -> ENU coordinate)
    3) setpoint로 천천히 접근


## safety_landing

마지막 경로점에 도달하여 Safety Landing하기 위한 패키지. PID 방식과 RL 방식으로 구성됨.

### Files

- [scripts](safety_landing/scripts): Directory containing the following scripts:
  - [PID_control_node.py](safety_landing/scripts/PID_control_node.py): Node for PID control of the drone during landing.
  - [vision_kalman_filter_node.py](safety_landing/scripts/vision_kalman_filter_node.py): Node for filtering vision data using Kalman filter.

**TODO**

- [x] RL_landing.py 테스트 후 CMakeLists.txt에 추가하기

## isly

isly package(I Still Love You)에서 착안. 미션 수행을 마치고 다시 HomePosition으로 되돌아 오는 패키지. `path_node`와 반대 순서로 경로점을 업데이트 해준다.

### Files
- [scripts](isly/scripts): Directory containing the following scripts:
  - [isly_path_node.py](isly/scripts/isly_path_node.py) : Node for planning and following a path while coming back home.

## utils
임무 수행 외에 필요한 기능들이 포함된 패키지로 패키지 이름은 `koreauav_utils`이다.
- [config](utils/config/)
  - [gps_coordinates.yaml](utils/config/gps_coordinates.yaml): 기체가 비행해야 하는 경로점을 GPS(LLA)에서 ENU로 간편하게 변환하기 위해 사용하는 YAML 파일.
- [launch](utils/launch/)
  - [yaml_to_gps.launch](utils/launch/yaml_to_gps.launch): YAML파일을 읽어 NavSatFix 메세지 형태로 publish하는 `yaml_to_gps`노드가 포함되어 있다.
  - [gps_to_enu.launch](utils/launch/gps_to_enu.launch): ASL 연구실에서 오픈소스로 작성한 [geodetic_utils](https://github.com/YonseiDrone/geodetic_utils)를 통해 NavSatFix를 ENU 좌표계로 변환한다.
- [scripts](utils/scripts/)
  - [cargo_launch.py](utils/scripts/cargo_launch.py): Pixhawk FC에서 PWM 신호를 통해 서보모터를 작동하기 위한 노드. 자세한 사항은 [README](utils/README.md) 참고.

## ysdrone_msgs


**DroneCommand.srv**
```srv
# Request
int32 command

---
# Response
bool res
string mode
```
