# Building Search

pointcloud에서 clustering된 물체를 베란다로 가정하고, centroid를 계산하여 최종 경로점과의 중간으로 기체를 이동시키는 노드

## Usage
```bash
rosrun building_search building_search_node

# Activate
rosservice call /drone_commane 2
```

## Code Overview

### BuildingSearch class

`BuildingSearch` 클래스는 최종 경로점에서 베란다에 조금 더 접근하고, 기체의 헤딩을 베란다 쪽으로 돌리도록 하는 클래스입니다.
`/local_pointcloud`와 `/mavros/local_position/pose` 토픽을 subscribe하고 Avoidance local planner에서 updateGoalCallback에 사용하는 `/input/goal_position`과 포지션 제어인 `/mavros/setpoint_position/local`을 publish합니다. 서비스 클라이언트와 서버를 사용하여 드론 명령과 상호 작용합니다.

클래스의 주요 기능은 `command`와 `cloud_cb` 메서드에 있습니다:

- `command`: 현재 임무 상태에 따라 드론을 타겟의 요로 돌리거나 드론을 타겟으로 이동시키는 등의 작업을 수행합니다. `/drone_command`에서 자신의 미션 번호와 동일한 경우만 제어를 시작합니다.

- `cloud_cb`: 들어오는 포인트 클라우드 데이터를 처리하여 클러스터링하고, 클러스터의 중심을 찾아 목표 목표 위치를 클러스터의 중심으로 설정합니다.

### ROS Callbacks and Services

`pose_cb`: Updates the current drone pose when a new pose message is received.

`srv_cb`: Sets the mission status when a new service request is received.