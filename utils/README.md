# koreauav_utils

### Config

- [gps_coordinates.yaml](./config/gps_coordinates.yaml): 실제 GPS LLA 좌표를 형식에 맞추어 입력.

### Launch

- [gps_to_enu.launch](./launch/gps_to_enu.launch): [geodetic_utils](https://github.com/YonseiDrone/geodetic_utils/)을 활용해 WGS84에서 UTM으로 변환해줌. **처음 기체에서 런치파일을 켤 때 들어오는 GPS값(`/mavros/global_position/global`)을 원점으로 사용함** (PixhawkFC 재부팅으로 만들어지는 `/mavros/local_position/pose`와 다름!)
- [yaml_to_gps.launch](./launch/yaml_to_gps.launch): config/gps_coordinates.yaml에서 읽어온 LLA 좌표를 `sensor_msgs::NavSatFix` 형식의 토픽으로 발행

### Scripts

- [cargo_launch.py](./scripts/cargo_launch.py):
	- Pixhawk FC에 PWM 시그널을 보내 서보모터를 제어하기 위한 노드
	- 대회에서 사용한 펌웨어 버전에서는 `MAV_CMD_DO_SET_ACTUATOR`를 지원.
	- [Servo Motor Parameters(YonseiDrone Notion)](https://www.notion.so/yonseidrone/Servo-Motor-Parameters-568f174a024c45d58006834091d93794?pvs=4)
	- v1.14 부터는 `GRIPPER`를 사용하는 방식으로 바뀌므로, 추후 업데이트 시 참고.
	- 아래와 같은 Reference들을 참고하여 코드를 작성하면 된다.
	- *Refernces*
		- [Ardupilot 문서: Servo Gripper](https://ardupilot.org/copter/docs/common-gripper-servo.html#servo-gripper)
		- [PX4 문서: Grippers](https://docs.px4.io/main/en/peripherals/gripper.html)
		- [MAVLink: MAV_CMD_DO_GRIPPER](https://mavlink.io/en/messages/common.html#MAV_CMD_DO_GRIPPER)
		- [사용하는 메세지: CommandLong](http://docs.ros.org/en/noetic/api/mavros_msgs/html/srv/CommandLong.html)

- [fligt_csv.py](./scripts/flight_csv.py): 23년 한국 항공기 경연대회 제출 CSV 파일을 자동으로 작성

### Etc
- [auto_service.py](./src/koreauav_utils/auto_service.py): 다음 미션으로 넘어가는 위해 사용하는 서비스 `/drone_command`를 여러 패키지에서 사용하기 위한 스크립트

### TODO
- [ ] : [flight_csv_reader.py](./scripts/flight_csv_rmse.py)
			작성된 제출용 csv 파일을 체크하는 파일. 경로점에 따른 색상은 잘 나오지만 수평, 수직 오차와 RMSE가 부정확
- [ ] : [flight_csv_rmse.py](./scripts/flight_csv_rmse.py)
			RMSE만 측정하는 스크립트. 경로점 번호가 출발~복귀마다 상이해서 이착륙 시에는 RMSE 측정 제외하는 것이 부정확함.