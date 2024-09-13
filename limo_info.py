#!/usr/bin/env python3
import rospy
from geometry_msgs.msg import PoseStamped
from nav_msgs.msg import Odometry
from std_msgs.msg import String
from cav_project.msg import limo_info, limo_info_array

class LimoInfoPublisher:
    def __init__(self):
        rospy.init_node('limo_info_publisher')
        self.info_pub = rospy.Publisher('/limo_info', limo_info, queue_size=10)
        self.limo_array_pub = rospy.Publisher('/limo_info_array', limo_info_array, queue_size=10)
        self.odom_sub = rospy.Subscriber('/odom', Odometry, self.odom_callback)
        self.mocap_sub = rospy.Subscriber('/vrpn_client_node/pose', PoseStamped, self.mocap_callback)
        self.task_sub = rospy.Subscriber('/task', String, self.task_callback)
        self.limo_data = limo_info()
        self.limo_data.ID.data = 1
        self.limo_array_data = limo_info_array()
        self.path_string = 'NONE'
        self.rate = rospy.Rate(10)

    def task_callback(self, msg):
        path = msg.data
        lane_s = path[1]
        turn_s = path[0]
        self.path_string = 'straight_' + lane_s
        self.limo_data.path = String(self.path_string)
        self.publish_info()

    def publish_info(self):
        self.info_pub.publish(self.limo_data)
        self.limo_array_data.limo_infos.append(self.limo_data)
        self.limo_array_pub.publish(self.limo_array_data)

    def odom_callback(self, msg):
        self.limo_data.vel.data = msg.twist.twist.linear.x
        self.publish_info()

    def mocap_callback(self, msg):
        if self.path_string == "NONE":
            return
        self.limo_data.x.data = msg.pose.position.x
        self.limo_data.y.data = msg.pose.position.y
        self.limo_data.origin_dist.data = msg.pose.position.z
        self.publish_info()

    def run(self):
        while not rospy.is_shutdown():
            self.rate.sleep()

if __name__ == '__main__':
    publisher = LimoInfoPublisher()
    publisher.run()
