#!/usr/bin/env python3
import rospy
import numpy as np
from geometry_msgs.msg import PoseStamped
from ackermann_msgs.msg import AckermannDrive
from scipy.integrate import odeint

class CAV:
    def __init__(self, node_name):
        self.node_name = node_name
        self.position_x = 0
        self.position_z = 0
        self.position_yaw = 0
        self.velocity = 0
        self.acceleration = 0
        self.Receivedata = 0
        self.kp = 0.05
        self.ki = 0.01
        self.kd = 0.05
        self.v_min = 0.15
        self.v_max = 1
        self.u_min = -10
        self.u_max = 3
        self.Delta_T = 0.05
        self.Lc = 0.0
        self.phiRearEnd = 1.2
        self.phiLateral = 1.0
        self.deltaSafetyDistance = 300
        self.max_steering_angle = 7000

        rospy.init_node(self.node_name, anonymous=True)
        self.sub = rospy.Subscriber('/vrpn_client_node/' + self.node_name + '/pose', PoseStamped, self.callback)
        self.pub = rospy.Publisher('vel_steer_' + self.node_name, AckermannDrive, queue_size=10)
        rospy.Rate(10)

    def callback(self, msg):
        self.position_z = msg.pose.position.z * 1000
        self.position_x = msg.pose.position.x * 1000
        self.position_yaw = 0
        self.Receivedata = 1

    def generate_map(self, isMain):
        self.right_top_x = 3044
        self.right_top_y = 2536
        self.right_center_x = 2040
        self.right_center_y = 2519
        self.right_bottom_x = 683
        self.right_bottom_y = 2536
        self.left_top_x = 2933
        self.left_top_y = -1980
        self.left_center_x = 1922
        self.left_center_y = -1977
        self.merging_pt_x = 1950
        self.merging_pt_y = -652
        self.lane_width = 450

        self.right_top_activation_range = (self.lane_width / 1.3, self.lane_width * 1.1)
        self.right_center_activation_range = (self.lane_width * 1.1, self.lane_width)
        self.right_bottom_activation_range = (self.lane_width * 1.2, self.lane_width)
        self.left_top_activation_range = (self.lane_width * 1.1, self.lane_width / 1.7)
        self.left_center_activation_range = (self.lane_width / 1.5, self.lane_width)
        self.merging_pt_activation_range = (self.lane_width, self.lane_width / 2)

        self.main_path = self.generate_line(self.right_center_x, self.right_center_y, self.left_center_x, self.left_center_y)
        self.merging_path = self.generate_line(self.right_bottom_x, self.right_bottom_y, self.merging_pt_x, self.merging_pt_y)

        self.merge_path_PID = (-0.0005, -0.00005, -0.001)
        self.main_path_PID = (-0.0005, -0.00005, -0.001)

        if isMain:
            self.points = [(self.right_center_x, self.right_center_y), (self.left_center_x, self.left_center_y)]
            self.lines = [self.main_path]
            self.dists = [self.calc_distance(self.right_center_x, self.right_center_y, self.left_center_x, self.left_center_y)]
            self.PIDs = [self.main_path_PID]
        else:
            self.points = [(self.right_bottom_x, self.right_bottom_y), (self.merging_pt_x, self.merging_pt_y)]
            self.lines = [self.merging_path]
            self.dists = [self.calc_distance(self.right_bottom_x, self.right_bottom_y, self.merging_pt_x, self.merging_pt_y)]
            self.PIDs = [self.merge_path_PID]

    def generate_line(self, x_1, y_1, x_2, y_2):
        A = -(y_2 - y_1)
        B = -(x_1 - x_2)
        C = -(y_1 * (x_2 - x_1) - (y_2 - y_1) * x_1)
        return A, B, C

    def calc_distance(self, x1, y1, x2, y2):
        distance = ((x1 - x2) ** 2 + (y1 - y2) ** 2) ** 0.5
        return distance

    def update_state(self, state, u, steering_angle):
        t_span = [0, self.Delta_T]
        solution = odeint(self.second_order_model, state, t_span, args=(u, steering_angle))
        return solution[-1]

    def second_order_model(self, x, t, u, steering_angle):
        dx = np.zeros(3)
        dx[0] = x[1]
        dx[1] = u
        dx[2] = steering_angle
        return dx

    def detect_collision_threat(self, other_CAV):
        distance = abs(self.position_z - other_CAV.position_z)
        if distance < self.deltaSafetyDistance:
            return True
        return False

    def normal_acceleration(self):
        return 0.4
