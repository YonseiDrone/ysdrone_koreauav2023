# offboard

- offboard package는 3개의 노드로 구성.

- [launch](offboard/launch)
  - [avoidance.launch](offboard/launch/avoidance.launch): Pixhawk FC와 Companion PC를 연결하고, RealSense455 카메라 구동, `control_node`를 함께 실행한다. 비행 전 구동되는 런치파일 중 하나.

  - [waypoint_server.launch](offboard/launch/waypoint_server.launch): 주어진 GPS값을 LOCAL_ENU 기준 좌표계로 변환해주고 `path_node`를 통해 기체가 해당 좌표점에 도달하였을 때 다음 경로점으로 목표점을 publish해준다. 플래너로 PX4-Avoidance를 사용하므로 `input/goal_position` 토픽의 메세지인 `MarkerArray`타입으로 발행한다.

  > [!IMPORTANT]\
  > 이 런치파일을 구동하였을 때는 경로점이 맞게 생성되었는지 반드시 확인해야 한다.
  > `path_node`에서 ros param으로 설정해주는 `destination_1_pose_x`, `destination_1_pose_y` ~ `destination_3_pose_x`, `destination_3_pose_y`와 `destination_z` 을 `rosparam get`으로 비행 전 체크해주어야 한다.
  > `bash view_destination.sh` 로 한번에 바로 확인 가능하다.

  - [mission.launch](offboard/launch/mission.launch): WPT#3 도달 후 수행하는 임무들(건물 탐색, 마커 인식, 화물 배송, 정밀 착륙)이 포함된 런치파일. `setmode_node`가 포함되어 있어 **앞선 런치파일들이 실행된 상태에서 런치되면 OFFBOARD 모드로 전환되므로 주의해야 한다.**

## control_node

- 다른 패키지에서 받은 정보를 종합하여 전체적으로 드론을 control
- Mission mode를 service call로 받아서 mode를 전환

## path_node

- 미리 설정된 waypoint들을 받아와서 rosparam으로 설정.
- 드론이 지정된 waypoint에 도달하면 goal 위치를 다음 waypoint로 업데이트.

## setmode_node

- 이륙할 때 FCU가 연결되었는지 확인하고 연결되었으면 드론을 Arming 시킴.
- 그 후, Offboard mode로 전환.
- 착륙할 때는 Auto.land mode로 전환하고 드론을 Disarming 시킴.
- INIT_ACTUATOR함수에서 서보모터에 service call을 주어 작동시킴.