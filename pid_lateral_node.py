#!/usr/bin/env python3
import rospy
from cav_project.msg import limo_info
from ackermann_msgs.msg import AckermannDrive

class PIDLateralController:
    def __init__(self):
        rospy.init_node('pid_lateral_controller', anonymous=True)
        self.cav_info_sub = rospy.Subscriber('/cav_info', limo_info, self.cav_info_callback)
        self.drive_pub = rospy.Publisher('vel_steer_cav', AckermannDrive, queue_size=10)
        self.cav_info = None
        self.kp = 0.0015
        self.ki = 0.000045
        self.kd = 0.0017
        self.rate = rospy.Rate(20)

    def cav_info_callback(self, msg):
        self.cav_info = msg

    def pid_lateral_controller(self, lateral_error, e_prev, e_int, delta_t):
        e_int += lateral_error * delta_t
        e_der = (lateral_error - e_prev) / delta_t
        steering_angle = self.kp * lateral_error + self.ki * e_int + self.kd * e_der
        steering_angle = max(min(steering_angle, 7000), -7000)
        return steering_angle, lateral_error, e_int

    def run(self):
        e_prev = 0
        e_int = 0
        delta_t = 0.05
        while not rospy.is_shutdown():
            if self.cav_info:
                lateral_error = -(self.cav_info.d1.data * self.cav_info.d2.data + self.cav_info.d2.data * self.cav_info.d2.data + self.cav_info.d2.data) / ((self.cav_info.d1.data**2 + self.cav_info.d2.data**2)**0.5)
                steering_angle, e_prev, e_int = self.pid_lateral_controller(lateral_error, e_prev, e_int, delta_t)
                drive_msg = AckermannDrive()
                drive_msg.speed = 0    # no longitudinal control
                drive_msg.steering_angle = steering_angle
                self.drive_pub.publish(drive_msg)
            self.rate.sleep()

if __name__ == '__main__':
    controller = PIDLateralController()
    controller.run()
