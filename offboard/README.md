# offboard

- offboard package는 3개의 노드로 구성.

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