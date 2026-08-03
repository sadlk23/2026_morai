#!/usr/bin/env python3
import rospy
from std_msgs.msg import Float64MultiArray, Float64, String, Int16
from geometry_msgs.msg import PointStamped

import matplotlib
matplotlib.use("TkAgg")
from matplotlib import pyplot as plt

import pandas as pd
import numpy as np
from matplotlib.patches import Circle


class PlottingNode(object):
	def __init__(self):
		# ROS2→ROS1: replaced Node init/rclpy.init with rospy.init_node
		self.node_name = 'plotting_node'
		# Parameters (ROS2 declare/get -> ROS1 param server if needed)
		# Example: self.some_param = rospy.get_param('~some_param', default_value)

		self.colors = ["red", "orange", "yellow", "green", "blue", "indigo", "violet"]

		self.mission_number = 0

		self.local_path = []
		self.local_yaw = []
		self.local_k = []
		self.mission_data = 0
		self.previous_mission_data = None

		self.car_odom = [0.0, 0.0]
		self.car_yaw = 0.0
		self.has_car_odom = False
		self.has_car_yaw = False
		self.trajectories = []
		self.dynamic_obstacle_utm = []
		self.dynamic_obstacle_update_time = None

		self.rp_s_static_point = []
		self.pl_small_static_point = []

		self.rp_b_static_point = []
		self.pl_big_static_point = []

		self.rp_prl_parking_point = []
		self.pl_parking_center_point = []

		self.rp_parking_edge_out_point = []
		self.pl_parking_edge_out_point = []

		self.rp_delivery_point = []

		self.rp_u_uturn_point = []
		self.pl_uturn_center_point = []

		# 협로
		self.rp_traffic_cone = []
		self.pl_traffic_cone = []
		self.pl_dwa_ref_point = []
		self.pl_global_path_point = []

		self.Lwall = []
		self.Rwall = []
		self.tunnel_small = []

		# 사선 주차
		self.rp_angled_parking_all_point = []
		self.pl_angled_parking_all_cluster = []
		self.pl_angled_parking_two_cluster = []
		self.pl_angled_parking_two_point = []
		self.pl_angled_parking_path_point = []

		#정지 포인트
		self.pl_stop_point = []
		self.pl_stop_distance = 0

		# 감속 거리
		self.pl_check_distance_point = []

		# KCIYT 디버깅
		self.valid_obstacle = []
		self.evade_point = []

		# ROS2→ROS1: replaced create_subscription
		self.pl_mission_number_sub = rospy.Subscriber('/Planning/mission', Int16, self.pl_mission_number_callback, queue_size=1)

		# 본선/예선
		self.pl_path_sub = rospy.Subscriber('/Planning/local_path', Float64MultiArray, self.lo_path_callback, queue_size=1)
		self.pl_yaw_sub = rospy.Subscriber('/Planning/path_yaw', Float64MultiArray, self.lo_yaw_callback, queue_size=1)
		self.pl_k_sub = rospy.Subscriber('/Planning/curvature', Float64MultiArray, self.lo_k_callback, queue_size=1)
		self.pl_small_static_utm_sub = rospy.Subscriber('/Planning/plot_small_static_utm', Float64MultiArray, self.pl_small_static_callback, queue_size=1)
		self.pl_big_static_utm_sub = rospy.Subscriber('/Planning/plot_big_static_utm', Float64MultiArray, self.pl_big_static_callback, queue_size=1)
		self.pl_parking_center_sub = rospy.Subscriber('/Planning/plot_parking_center', Float64MultiArray, self.pl_parking_center_callback, queue_size=1)
		self.pl_parking_edge_out_sub = rospy.Subscriber('/Planning/plot_parking_edge_out', Float64MultiArray, self.pl_parking_edge_out_callback, queue_size=1)
		self.pl_uturn_center_sub = rospy.Subscriber('/Planning/plot_u_turn_point_utm', Float64MultiArray, self.pl_uturn_center_callback, queue_size=1)

		self.lo_c_utm_sub = rospy.Subscriber('/Local/utm', PointStamped, self.lo_c_UTM_callback, queue_size=1)
		self.lo_c_yaw_sub = rospy.Subscriber('/Local/heading', Float64, self.lo_c_yaw_callback, queue_size=1)
		self.dynamic_obstacle_sub = rospy.Subscriber('/LiDAR/dynamic_obstacle_pos', Float64MultiArray, self.dynamic_obstacle_callback, queue_size=1)

		self.rp_small_object_sub = rospy.Subscriber('/Convert/small_object_UTM', Float64MultiArray, self.rp_small_cone_UTM_callback, queue_size=1)
		self.rp_u_turn_point_sub = rospy.Subscriber('/Convert/u_turn_point', Float64MultiArray, self.rp_uturn_point_callback, queue_size=1)
		self.rp_big_object_sub = rospy.Subscriber('/Convert/big_object_UTM', Float64MultiArray, self.rp_big_cone_UTM_callback, queue_size=1)
		self.rp_prl_points_sub = rospy.Subscriber('/Convert/prl_points', Float64MultiArray, self.rp_prl_parking_point_callback, queue_size=1)
		self.rp_prl_edge_out_sub = rospy.Subscriber('/Convert/parking_edge', Float64MultiArray, self.rp_prl_parking_edge_out_callback, queue_size=1)
		self.rp_delivery_point_sub = rospy.Subscriber('/Convert/deli_UTM', Float64MultiArray, self.rp_delivery_point_callback, queue_size=1)

		self.file_path_sub = rospy.Subscriber('Planning/plot_file_path', String, self.file_path_callback, queue_size=1)
		self.trajectory_sub = rospy.Subscriber('/Planning/plot_all_path', Float64MultiArray, self.trajectory_callback, queue_size=1)

		# KCIYT 디버깅
		self.valid_obstacle_sub = rospy.Subscriber('/Planning/plot_tunnel_valid_obstacle', Float64MultiArray, self.tunnel_valid_obstacle_callback, queue_size=1)
		self.evade_point_sub = rospy.Subscriber('/Planning/plot_tunnel_evade_point', Float64MultiArray, self.tunnel_evade_point_callback, queue_size=1)

		# 터널
		self.tunnel_L_wall_sub = rospy.Subscriber('/Planning/plot_tunnel_L_wall', Float64MultiArray, self.tunnel_L_wall_callback, queue_size=1)
		self.tunnel_R_wall_sub = rospy.Subscriber('/Planning/plot_tunnel_R_wall', Float64MultiArray, self.tunnel_R_wall_callback, queue_size=1)
		self.tunnel_small_static_sub = rospy.Subscriber('/Planning/plot_tunnel_small_static', Float64MultiArray, self.pl_tunnel_small_callback, queue_size=1)

		# 협로
		self.pl_traffic_cone_sub = rospy.Subscriber('/Planning/plot_traffic_cone_utm', Float64MultiArray, self.pl_traffic_cone_callback, queue_size=1)
		self.rp_traffic_cone_sub = rospy.Subscriber('/Planning/narrow_object_UTM', Float64MultiArray, self.rp_traffic_cone_callback, queue_size=1)
		self.pl_dwa_ref_point_sub = rospy.Subscriber('/Planning/plot_dwa_ref_point', Float64MultiArray, self.pl_dwa_ref_point_callback, queue_size=1)
		self.pl_global_path_point_sub = rospy.Subscriber('/Planning/plot_global_path_point', Float64MultiArray, self.pl_global_path_point_callback, queue_size=1)

		# 사선 주차
		self.rp_angled_parking_all_point_sub = rospy.Subscriber('/Convert/angled_parking_all_point_UTM', Float64MultiArray, self.rp_angled_parking_all_point_callback, queue_size=1)
		self.pl_angled_parking_all_cluster_sub = rospy.Subscriber('/Planning/plot_angled_parking_all_cluster', Float64MultiArray, self.pl_angled_parking_all_cluster_callback, queue_size=1)
		self.pl_angled_parking_two_cluster_sub = rospy.Subscriber('/Planning/plot_angled_parking_two_cluster', Float64MultiArray, self.pl_angled_parking_two_cluster_callback, queue_size=1)
		self.pl_angled_parking_two_point_sub = rospy.Subscriber('/Planning/plot_angled_parking_two_point', Float64MultiArray, self.pl_angled_parking_two_point_callback, queue_size=1)
		self.pl_angled_parking_path_point_sub = rospy.Subscriber('/Planning/plot_angled_parking_path_point', Float64MultiArray, self.pl_angled_parking_path_point_callback, queue_size=1)

		#정지 포인트
		self.pl_stop_point_sub = rospy.Subscriber('/Planning/plot_stop_point', Float64MultiArray, self.pl_stop_point_callback, queue_size=1)
		self.pl_stop_distance_sub = rospy.Subscriber('/Planning/plot_stop_distance', Float64, self.pl_stop_distance_callback, queue_size=1)

		# 감속 거리
		self.pl_check_distance_sub = rospy.Subscriber('/Planning/plot_check_distance_point', Float64MultiArray, self.pl_check_distance_point_callback, queue_size=1)

		self.global_path = pd.DataFrame()

		# ROS2→ROS1: Timer changed, added 'event' argument
		self.timer = rospy.Timer(rospy.Duration.from_sec(0.01), self.draw_plot)

	def read_columns(self, file_path):
		try:
			df = pd.read_csv(file_path, delimiter=',', header=None)
			columns = df.iloc[:, [0, 1]]
			return columns
		except Exception as e:
			rospy.logerr(f"파일 읽기 오류: {e}")
			return pd.DataFrame()

	def draw_plot(self, event):
		# ROS2→ROS1: logging API changed
		rospy.loginfo("그래프 그리는 중...")
		plt.clf()
		plt.gcf().canvas.mpl_connect('key_release_event', lambda event: [exit(0) if event.key == 'escape' else None])
		plt.axis("equal")

		center_x, center_y = self.car_odom[0], self.car_odom[1]

		if self.mission_number == 23:
			center_x, center_y = 0, 0
		else:
			center_x, center_y = self.car_odom[0], self.car_odom[1]

		padding = 20
		arrow_length = 2.0
		h_width = 1.0
		h_length = 1.0

		if self.mission_number == 999 or self.mission_number == 998:
			padding = 10
			arrow_length = 2.0
			h_width = 1.0
			h_length = 1.0
		else:
			padding = 30
			arrow_length = 4.0
			h_width = 2.0
			h_length = 2.0

		tn_padding_m01 = padding - 2 if self.mission_number == 23 else 0
		tn_padding_m02 = padding - 10 if self.mission_number == 23 else 0

		plt.xlim(center_x - padding + tn_padding_m01, center_x + padding - tn_padding_m02)
		plt.ylim(center_y - padding + tn_padding_m02, center_y + padding - tn_padding_m02)

		if self.valid_obstacle:
			x_vals_0 = []
			y_vals_0 = []
			x_vals_1 = []
			y_vals_1 = []

			for i in range(0, len(self.valid_obstacle), 3):
				x = self.valid_obstacle[i]
				y = self.valid_obstacle[i + 1]
				direction = self.valid_obstacle[i + 2]

				if direction == 0:
					x_vals_0.append(x)
					y_vals_0.append(y)
				elif direction == 1:
					x_vals_1.append(x)
					y_vals_1.append(y)

			if x_vals_0 and y_vals_0:
				plt.plot(x_vals_0, y_vals_0, 'o', color='green', markersize=5, label='Direction 0')

			if x_vals_1 and y_vals_1:
				plt.plot(x_vals_1, y_vals_1, 'o', color='gray', markersize=5, label='Direction 1')

		if self.evade_point:
			plt.plot(self.evade_point[0::2], self.evade_point[1::2], 'or', color='purple', markersize=5)

		if self.trajectories:
			plt.plot(self.trajectories[0::2], self.trajectories[1::2], 'or', color='orange', markersize=0.2, linewidth=0.1)

		if self.local_path:
			plt.plot(self.local_path[0::2], self.local_path[1::2], 'ob', markersize=1.3, linewidth=0.5)
		plt.plot(self.car_odom[0], self.car_odom[1], 'og')

		if self.mission_number == 100 and len(self.pl_global_path_point) >= 2:
			start_x = self.pl_global_path_point[0]
			start_y = self.pl_global_path_point[1]
			plt.plot(start_x, start_y, marker='X', color='magenta',
					 markersize=11, linestyle='None', zorder=5)
			plt.annotate('LANE CHANGE START', (start_x, start_y),
						 xytext=(6, 6), textcoords='offset points',
						 color='magenta', fontsize=8)

		if self.mission_number == 100 and len(self.pl_global_path_point) >= 4:
			slow_x = self.pl_global_path_point[2]
			slow_y = self.pl_global_path_point[3]
			plt.plot(slow_x, slow_y, marker='D', color='cyan',
					 markersize=9, linestyle='None', zorder=5)
			plt.annotate('HIGH-PASS 60', (slow_x, slow_y),
						 xytext=(6, 6), textcoords='offset points',
						 color='darkcyan', fontsize=8)

		if self.rp_s_static_point:
			plt.plot(self.rp_s_static_point[0::2], self.rp_s_static_point[1::2], 'or', color='orange')

		if self.pl_small_static_point:
			plt.plot(self.pl_small_static_point[0::2], self.pl_small_static_point[1::2], 'o', color='red', markersize=5)

		if self.rp_b_static_point:
			plt.plot(self.rp_b_static_point[0::2], self.rp_b_static_point[1::2], 'or', color='orange')

		if self.pl_big_static_point:
			plt.plot(self.pl_big_static_point[0::2], self.pl_big_static_point[1::2], 'o', color='red', markersize=5)

		# LiDAR Standard의 동적 장애물 중심점. DynamicObstacleStop이 사용하는
		# Velodyne->UTM 변환과 동일한 계산 결과를 표시한다.
		if (self.dynamic_obstacle_utm and
				self.dynamic_obstacle_update_time is not None and
				(rospy.Time.now() - self.dynamic_obstacle_update_time).to_sec() <= 0.5):
			plt.plot(self.dynamic_obstacle_utm[0::2],
					 self.dynamic_obstacle_utm[1::2],
					 linestyle='None', marker='X', color='magenta',
					 markeredgecolor='black', markersize=11,
					 label='LiDAR dynamic obstacle')

		if self.rp_prl_parking_point:
			plt.plot(self.rp_prl_parking_point[0::2], self.rp_prl_parking_point[1::2], 'or', color='orange')

		if self.pl_parking_center_point:
			plt.plot(self.pl_parking_center_point[0::2], self.pl_parking_center_point[1::2], 'o', color='red', markersize=5)

		if self.pl_parking_edge_out_point:
			plt.plot(self.pl_parking_edge_out_point[0::2], self.pl_parking_edge_out_point[1::2], 'or', color='orange')

		if self.rp_parking_edge_out_point:
			plt.plot(self.rp_parking_edge_out_point[0::2], self.rp_parking_edge_out_point[1::2], 'o', color='red', markersize=5)

		if self.rp_u_uturn_point:
			plt.plot(self.rp_u_uturn_point[0::2], self.rp_u_uturn_point[1::2], 'or', color='orange')

		if self.pl_uturn_center_point:
			plt.plot(self.pl_uturn_center_point[0::2], self.pl_uturn_center_point[1::2], 'o', color='red', markersize=5)

		if self.rp_delivery_point:
			x_coords = self.rp_delivery_point[0::3]
			y_coords = self.rp_delivery_point[1::3]
			plt.plot(x_coords, y_coords, 'o', color='red', markersize=5)

		# 사선 주차
		if self.rp_angled_parking_all_point:
			plt.plot(self.rp_angled_parking_all_point[0::2], self.rp_angled_parking_all_point[1::2], 'or', color='orange')

		if self.pl_angled_parking_all_cluster:
			plt.plot(self.pl_angled_parking_all_cluster[0::2], self.pl_angled_parking_all_cluster[1::2], 'o', color='red', markersize=5)

		if self.pl_angled_parking_two_point:
			plt.plot(self.pl_angled_parking_two_point[0::2], self.pl_angled_parking_two_point[1::2], 'o', color='green', markersize=5)
		elif self.pl_angled_parking_two_cluster:
			plt.plot(self.pl_angled_parking_two_cluster[0::2], self.pl_angled_parking_two_cluster[1::2], 'o', color='red', markersize=5)

		if self.pl_angled_parking_path_point:
			xs = self.pl_angled_parking_path_point[0::2]
			ys = self.pl_angled_parking_path_point[1::2]

			for i, (x, y) in enumerate(zip(xs, ys)):
				c = self.colors[i % len(self.colors)]
				plt.plot(x, y, 'o', color=c, markersize=10)

		# 정지 포인트
		if self.pl_stop_point and self.pl_stop_distance:
			ax = plt.gca()
			x = self.pl_stop_point[0]
			y = self.pl_stop_point[1]
			circle = Circle((x, y), radius=self.pl_stop_distance, fill=False, edgecolor='red', linewidth=1)
			ax.add_patch(circle)
			plt.plot(x, y, 'o', color='red', markersize=1)

		# 감속 거리
		if self.pl_check_distance_point:
			plt.plot([self.pl_check_distance_point[0], self.pl_check_distance_point[1]],
					 [self.pl_check_distance_point[2], self.pl_check_distance_point[3]],
					 color='black', linewidth=3)

		# 협로 관련
		if len(self.Lwall) > 1:
			x = self.Lwall[0::2]
			y = self.Lwall[1::2]
			min_len = min(len(x), len(y))
			plt.plot(x[:min_len], y[:min_len], color='black', linewidth=3)

		if len(self.Rwall) > 1:
			x = self.Rwall[0::2]
			y = self.Rwall[1::2]
			min_len = min(len(x), len(y))
			plt.plot(x[:min_len], y[:min_len], color='black', linewidth=3)

		# rp_traffic_cone 연한 색
		if self.rp_traffic_cone:
			for i in range(0, len(self.rp_traffic_cone), 3):
				x = self.rp_traffic_cone[i]
				y = self.rp_traffic_cone[i+1]
				cls = int(self.rp_traffic_cone[i+2])
				if cls == 1:
					color = 'lightblue'
				elif cls == 2:
					color = 'orange'
				else:
					color = 'gray'
				plt.plot(x, y, 'or', color=color)

		# pl_traffic_cone 진한 색
		if self.pl_traffic_cone:
			for i in range(0, len(self.pl_traffic_cone), 3):
				x = self.pl_traffic_cone[i]
				y = self.pl_traffic_cone[i+1]
				cls = int(self.pl_traffic_cone[i+2])
				if cls == 1:
					color = 'blue'
				elif cls == 2:
					color = 'yellow'
				else:
					color = 'black'
				plt.plot(x, y, 'o', color=color, markersize=5)

		plt.arrow(self.car_odom[0], self.car_odom[1],
				 4.0 * np.cos(self.car_yaw), 4.0 * np.sin(self.car_yaw),
				  head_width=2.0, head_length=2.0, fc='g', ec='g')

		plt.grid(True)
		plt.pause(0.001)

	def file_path_callback(self, msg):
		if self.previous_mission_data != msg.data:
			self.mission_data = msg.data
			self.previous_mission_data = self.mission_data
			path = f'{self.mission_data}.txt'
			self.global_path = self.read_columns(path)
			if self.global_path.empty:
				rospy.logerr(f"{path}에서 데이터를 불러오는데 실패했습니다.")
			else:
				rospy.loginfo(f"{path}에서 새로운 미션 데이터를 불러왔습니다.")

	def pl_mission_number_callback(self, msg):
		self.mission_number = msg.data
		if self.mission_number == 23:
			self.local_path = []
			self.tunnel_small = []

	def lo_path_callback(self, msg):
		self.local_path = msg.data

	def lo_yaw_callback(self, msg):
		self.local_yaw = msg.data

	def lo_k_callback(self, msg):
		self.local_k = msg.data

	def lo_c_UTM_callback(self, msg):
		self.car_odom = [msg.point.x, msg.point.y]
		self.has_car_odom = True

	def lo_c_yaw_callback(self, msg):
		self.car_yaw = msg.data
		self.has_car_yaw = True

	def dynamic_obstacle_callback(self, msg):
		# Empty array는 현재 프레임에서 장애물이 없음을 의미한다.
		self.dynamic_obstacle_utm = []
		self.dynamic_obstacle_update_time = rospy.Time.now()
		if not self.has_car_odom or not self.has_car_yaw or len(msg.data) < 2:
			return

		cos_yaw = np.cos(self.car_yaw)
		sin_yaw = np.sin(self.car_yaw)
		for index in range(0, len(msg.data) - 1, 2):
			sensor_x = msg.data[index]
			sensor_y = msg.data[index + 1]
			if not np.isfinite(sensor_x) or not np.isfinite(sensor_y):
				continue

			utm_x = self.car_odom[0] + sensor_x * cos_yaw - sensor_y * sin_yaw
			utm_y = self.car_odom[1] + sensor_x * sin_yaw + sensor_y * cos_yaw
			self.dynamic_obstacle_utm.extend([utm_x, utm_y])

	def trajectory_callback(self, msg):
		self.trajectories = msg.data

	# Planning
	def pl_small_static_callback(self, msg):
		self.pl_small_static_point = msg.data

	def pl_parking_center_callback(self, msg):
		self.pl_parking_center_point = msg.data

	def pl_parking_edge_out_callback(self, msg):
		self.pl_parking_edge_out_point = msg.data

	def pl_uturn_center_callback(self, msg):
		self.pl_uturn_center_point = msg.data

	def pl_big_static_callback(self, msg):
		self.pl_big_static_point = msg.data

	def pl_traffic_cone_callback(self, msg):
		self.pl_traffic_cone = msg.data

	def pl_global_path_point_callback(self, msg):
		self.pl_global_path_point = msg.data

	# 사선 주차
	def rp_angled_parking_all_point_callback(self, msg):
		self.rp_angled_parking_all_point = msg.data

	def pl_angled_parking_all_cluster_callback(self, msg):
		self.pl_angled_parking_all_cluster = msg.data

	def pl_angled_parking_two_cluster_callback(self, msg):
		self.pl_angled_parking_two_cluster = msg.data

	def pl_angled_parking_two_point_callback(self, msg):
		self.pl_angled_parking_two_point = msg.data

	def pl_angled_parking_path_point_callback(self, msg):
		self.pl_angled_parking_path_point = msg.data

	# 정지 포인트
	def pl_stop_point_callback(self, msg):
		self.pl_stop_point = msg.data

	def pl_stop_distance_callback(self, msg):
		self.pl_stop_distance = msg.data

	# 감속 거리
	def pl_check_distance_point_callback(self, msg):
		self.pl_check_distance_point = msg.data

	# Convert
	def rp_small_cone_UTM_callback(self, msg):
		self.rp_s_static_point = msg.data

	def rp_big_cone_UTM_callback(self, msg):
		self.rp_b_static_point = msg.data

	def rp_prl_parking_point_callback(self, msg):
		self.rp_prl_parking_point = msg.data

	def rp_prl_parking_edge_out_callback(self, msg):
		self.rp_parking_edge_out_point = msg.data

	def rp_uturn_point_callback(self, msg):
		self.rp_u_uturn_point = msg.data

	def rp_traffic_cone_callback(self, msg):
		self.rp_traffic_cone = msg.data

	def rp_delivery_point_callback(self, msg):
		self.rp_delivery_point = msg.data

	# 터널 콜백
	def tunnel_L_wall_callback(self, msg):
		self.Lwall = msg.data

	def tunnel_R_wall_callback(self, msg):
		self.Rwall = msg.data

	def pl_tunnel_small_callback(self, msg):
		self.tunnel_small = msg.data

	def pl_dwa_ref_point_callback(self, msg):
		self.pl_dwa_ref_point = msg.data

	# KCITY 디버깅
	def tunnel_valid_obstacle_callback(self, msg):
		self.valid_obstacle = msg.data

	def tunnel_evade_point_callback(self, msg):
		self.evade_point = msg.data


def main():
	# ROS2→ROS1: rclpy.init/Node/spin->rospy.init_node/rospy.spin
	rospy.init_node('plotting_node')
	node = PlottingNode()
	try:
		rospy.spin()
	except KeyboardInterrupt:
		pass


if __name__ == '__main__':
	main()
