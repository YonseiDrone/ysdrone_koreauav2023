# 2023 koreauav Khadas VIM4 main-repo
**Autor: Jusuk Lee, ChanJoon Park, Inha Baek**

## offboard

Offboard 모드에서 드론을 제어하기 위한 패키지.
### Files

- [scripts](offboard/scripts): Directory containing the following scripts:
  - [control_node.py](offboard/scripts/control_node.py): Node for controlling the drone.
  - [path_node.py](offboard/scripts/path_node.py): Node for planning and following a path.
  - [setmode_node.py](offboard/scripts/setmode_node.py): Node for setting the flight mode of the drone.

**TODO**

- [ ] GPS 좌표계 yaml 파일을 읽어서 업데이트 해줘야 하는 경로에 추가
- [ ] cpp 버전으로 작성

## safety_landing

마지막 경로점에 도달하여 Safety Landing하기 위한 패키지. PID 방식과 RL 방식으로 구성됨.

### Files

- [scripts](safety_landing/scripts): Directory containing the following scripts:
  - [PID_control_node.py](safety_landing/scripts/PID_control_node.py): Node for PID control of the drone during landing.
  - [vision_kalman_filter_node.py](safety_landing/scripts/vision_kalman_filter_node.py): Node for filtering vision data using Kalman filter.

**TODO**

- [ ] RL_landing.py 테스트 후 CMakeLists.txt에 추가하기

## isly

isly package(I Still Love You)에서 착안. 미션 수행을 마치고 다시 HomePosition으로 되돌아 오는 패키지.

### Files
- [scripts](isly/scripts): Directory containing the following scripts:
  - [isly_path_node.py](isly/scripts/isly_path_node.py) : Node for planning and following a path while coming back home.

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
