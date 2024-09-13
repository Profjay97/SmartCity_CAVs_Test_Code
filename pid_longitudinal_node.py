#!/usr/bin/env python3
import rospy
from cav_project.msg import limo_info, QP_solution
from ackermann_msgs.msg import AckermannDrive

class PIDLongitudinalController:
    def __init__(self):
        rospy.init_node('pid_longitudinal_controller', anonymous=True)
        self.qp_solution_sub = rospy.Subscriber('/qp_solution', QP_solution, self.qp_solution_callback)
        self.cav_info_sub = rospy.Subscriber('/cav_info', limo_info, self.cav_info_callback)
        self.drive_pub = rospy.Publisher('vel_steer_cav', AckermannDrive, queue_size=10)
        self.qp_solution = None
        self.cav_info = None
        self.kp = 0.1
        self.ki = 0.01
        self.kd = 0.05
        self.rate = rospy.Rate(20)

    def cav_info_callback(self, msg):
        self.cav_info = msg

    def qp_solution_callback(self, msg):
        self.qp_solution = msg

    def pid_longitudinal_controller(self, desired_velocity, actual_velocity, e_prev, e_int, delta_t):
        error = desired_velocity - actual_velocity
        e_int += error * delta_t
        e_der = (error - e_prev) / delta_t
        control_input = self.kp * error + self.ki * e_int + self.kd * e_der
        control_input = max(min(control_input, 1), -1)
        return control_input, error, e_int

    def run(self):
        e_prev = 0
        e_int = 0
        delta_t = 0.05
        while not rospy.is_shutdown():
            if self.qp_solution and self.cav_info:
                desired_velocity = self.cav_info.vel.data + self.qp_solution.u.data * 0.05
                actual_velocity = self.cav_info.vel.data
                control_input, e_prev, e_int = self.pid_longitudinal_controller(desired_velocity, actual_velocity, e_prev, e_int, delta_t)
                drive_msg = AckermannDrive()
                drive_msg.speed = control_input
                drive_msg.steering_angle = 0  # no lateral control
                self.drive_pub.publish(drive_msg)
            self.rate.sleep()

if __name__ == '__main__':
    controller = PIDLongitudinalController()
    controller.run()
