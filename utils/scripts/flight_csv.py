#!/usr/bin/env python

import rospy
from std_msgs.msg import Float32
from sensor_msgs.msg import NavSatFix
from geometry_msgs.msg import PoseStamped
from mavros_msgs.msg import State
import csv
from datetime import datetime

import sys
import signal

def signal_handler(signal, frame): # ctrl + c -> exit program
	print('You pressed Ctrl+C!')
	sys.exit(0)
signal.signal(signal.SIGINT, signal_handler)

class FlightDataRecorder:
### Flight Data Format
# | Auto/Manual | Waypoint Flag | GPS Time | Latitude | Longitude | Altitude |

### Waypoint(Event) Flag
# Home-WPT1 : 			1
# WPT1-WPT2 : 			2
# WPT2-WPT3: 			3
# WPT3-CargoMission : 	4
# CargoMission-WPT3 : 	3
# WPT3-WPT2 : 			2
# WPT2-WPT1 : 			1
# WPT1-Home : 			0

	def __init__(self):
		rospy.init_node("flight_csv")
		self.current_state = State()
		self.mission = Float32()
		self.local_pose = PoseStamped()
		self.destination_cnt = 0

		self.state_sub = rospy.Subscriber("/mavros/state", State, self.state_cb)
		self.mission_sub = rospy.Subscriber("/mission", Float32, self.mission_cb)
		self.destination_cnt_sub = rospy.Subscriber("/destination_cnt", Float32, self.destination_cnt_cb)
		self.local_pose_sub = rospy.Subscriber("/mavros/local_position/pose", PoseStamped, self.local_pose_cb)
		self.gps_sub = rospy.Subscriber("/mavros/global_position/global", NavSatFix, self.gps_cb)
		
		self.DEBUG = False
		self.datetime = datetime.now()
		rospy.loginfo("[CSV] Writing...")
		with open(f"yonsei_drone_flight_log_{self.datetime}.csv", "a") as csv_file:
			csv_writer = csv.writer(csv_file)
			csv_writer.writerow(["자동, 수동", "경로점", "GPS Time", "Latitude", "Longitude", "Altitude"])

	def state_cb(self, msg):
		# Avg. rate: 1.02 Hz
		self.current_state = msg

	def mission_cb(self, msg):
		self.mission = msg

	def local_pose_cb(self, msg):
		# Avg. rate: 30 Hz
		self.local_pose = msg

	def destination_cnt_cb(self, msg):
		self.destination_cnt = msg.data
  
	def gps_cb(self, msg):
		# Avg. rate: 10 Hz
		auto_mode = 1 if self.current_state.mode == "OFFBOARD" else 0
		mode_time = self.current_state.header.stamp.to_sec()
		event_flag = self.get_event_flag(self.mission.data)
		# Instead of using Altitude(WGS 84), use AGL(Above Ground Level)
		pose_time = self.local_pose.header.stamp.to_sec()
		local_z = self.local_pose.pose.position.z

		gps_time = msg.header.stamp.to_sec()
		latitude = msg.latitude
		longitude = msg.longitude
		altitude = msg.altitude

		mode_gps_diff = mode_time - gps_time
		pose_gps_diff = pose_time - gps_time
		if(self.DEBUG):
			with open(f"yonsei_drone_flight_log_{self.datetime}.csv", "a") as csv_file:
				csv_writer = csv.writer(csv_file)
				csv_writer.writerow([auto_mode, event_flag, gps_time, latitude, longitude, local_z, mode_gps_diff, pose_gps_diff])
		else:
			with open(f"yonsei_drone_flight_log_{self.datetime}.csv", "a") as csv_file:
				csv_writer = csv.writer(csv_file)
				csv_writer.writerow([auto_mode, event_flag, gps_time, latitude, longitude, local_z])

	def record(self):
		rospy.spin()
  
	def get_event_flag(self, mission_data):
		# TODO: WPT에 따른 mission_data 필요. 현재는 Avoidance가 모두 1으로 나오고 있음.
		event_flag = 1
		# Takeoff Mode
		if mission_data == 0:
			event_flag = 1
		# Avoidance Mode
		elif mission_data == 1:
			# WPT_1
			if self.destination_cnt == 0:
				event_flag = 1
			# WPT_2
			elif self.destination_cnt == 1:
				event_flag = 2
			# WPT_3
			elif self.destination_cnt == 2:
				event_flag = 3
		# Building Search Mode ~ Cargo Launch Mode
		elif mission_data in [2, 3, 4]:
			event_flag = 4
		# Of course I Still Love You
		elif mission_data == 5:
			if self.destination_cnt == 0:
				event_flag = 3
			elif self.destination_cnt == 1:
				event_flag = 2
			elif self.destination_cnt == 2:
				event_flag = 1
		# Landing(Position, RL, AruCo) to Disarming
		elif mission_data in [6, 8, 9, 10, 11]:
			event_flag = 0
			
		return event_flag

if __name__ == "__main__":
	try:
		recorder = FlightDataRecorder()
		recorder.record()
	except (rospy.ROSInterruptException, SystemExit, KeyboardInterrupt):
		sys.exit()
