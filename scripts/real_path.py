#!/usr/bin/python3
import rospy 

from nav_msgs.msg import Path, Odometry
from geometry_msgs.msg import PoseStamped

def base_pose_real_path(data):
    global path_real
    path_real.header = data.header
    pose = PoseStamped()
    pose.header = data.header
    pose.pose = data.pose.pose
    path_real.poses.append(pose)
    real_path_publisher.publish(path_real)

if __name__ == "__main__":
    rospy.init_node("realpath")
    path_real = Path()
    real_subscriber = rospy.Subscriber('/base_pose_ground_truth', Odometry, base_pose_real_path)
    real_path_publisher = rospy.Publisher('/realpath', Path, queue_size=10)
    rospy.spin()