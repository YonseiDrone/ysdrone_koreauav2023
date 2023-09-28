# Building Search

### building_search node (c++) (Deprecated)

> [!WARNING]  
> 대회에서는 C++ 파일들을 사용하지 않았으며, PX4-Avoidance 원본 패키지의 `/input/goal_position`로 제어하지 않습니다. 대회용 수정된 버전의 PX4-Avoidance를 사용하였으므로 자세한 사항은 [링크](https://github.com/YonseiDrone/PX4-Avoidance)를 참고해주세요.

**Include**
- [building_search.hpp](./include/building_search/building_search.hpp): Class definition header file

**Src**
- [building_search_node.cpp](./src/building_search_node.cpp): ROS node "building_search_node" declaration
- [building_search.cpp](./src/building_search.cpp): Class Implementations
	- `BuildingSearch` 는 최종 경로점 WPT#3에서 베란다의 위치를 파악하고 마커가 보이도록 접근하는 목적을 가진 패키지입니다.
	- 클래스의 주요 기능은 `command`와 `cloud_cb` 에 있습니다
	- `command`: 현재 임무 상태에 따라 드론을 타겟의 요로 돌리거나 드론을 타겟으로 이동시키는 등의 작업을 수행합니다. `/drone_command`에서 자신의 미션 번호와 동일한 경우만 제어를 시작합니다.
	- `cloud_cb`: Avoidance에서 들어오는 포인트 클라우드 `local_pointcloud` 데이터를 처리하여 클러스터링하고, 클러스터의 중심을 찾아 목표 위치를 클러스터의 중심으로 설정합니다. 또한 RViz 시각화를 위해 건물 중심 좌표를 `Marker`로 publish합니다.

### building_search_python_node

**Scripts**
- [building_search_python_node.py](./scripts/building_search_python_node.py):
	- PCL 대신 [python-pcl](https://github.com/strawlab/python-pcl)을 사용하여 구현되었으며, c++버전에서 몇 가지 기능이 추가된 노드
	- Timer 함수 없이 `cloud_cb`에서 주요 기능들이 모두 이루어지고 여기서 publish하는 `/building/search/target_pose`토픽은 control_node에서 에서 받아 제어됨.

### TODO
- [ ] : python으로 작성된 최신 버전 노드를 cpp로 수정(pointcloud 처리에 c++이 우수하며, 유지보수되고 있는 PCL을 사용해야함)