## [cargo_launch.py](./scripts/cargo_launch.py)

Pixhawk FC에 PWM 시그널을 보내 서보모터를 제어하기 위한 노드인 `cargo_launch`를 README에서 추가 설명함.

PX4 공식 문서나 `mavros`에서 소개되는 Actuator는 주로 BLDC 모터를 지칭하는데 혼동되기 쉬워 적절한 토픽이나 서비스를 사용하지 못함.

서보모터를 제어할 때는 `GRIPPER`를 사용하는 방식으로 사용하지만 공식 문서에서도 테스트 되었다고 나오진 않음.

따라서 아래와 같은 Reference들을 참고하여 코드를 작성하면 된다.

- [Ardupilot 문서: Servo Gripper](https://ardupilot.org/copter/docs/common-gripper-servo.html#servo-gripper)

- [PX4 문서: Grippers](https://docs.px4.io/main/en/peripherals/gripper.html)

- [MAVLink: MAV_CMD_DO_GRIPPER](https://mavlink.io/en/messages/common.html#MAV_CMD_DO_GRIPPER)

- [사용하는 메세지: CommandLong](http://docs.ros.org/en/noetic/api/mavros_msgs/html/srv/CommandLong.html)