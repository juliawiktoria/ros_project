#!/usr/bin/python3
import rospy 

from nav_msgs.msg import Path, Odometry
from geometry_msgs.msg import PoseStamped

def odom_path(data):
    global path_odom
    path_odom.header = data.header
    pose = PoseStamped()
    pose.header = data.header
    pose.pose = data.pose.pose
    path_odom.poses.append(pose)
    odom_path_publisher.publish(path_odom)


if __name__ == "__main__":
    rospy.init_node("odompath")
    path_odom = Path()
    odom_subscriber = rospy.Subscriber('/odom', Odometry, odom_path)
    odom_path_publisher = rospy.Publisher('/odompath', Path, queue_size=10)
    rospy.spin()