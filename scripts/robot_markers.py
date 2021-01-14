#!/usr/bin/python3

import rospy
from visualization_msgs.msg import Marker, MarkerArray
from geometry_msgs.msg import Pose
from nav_msgs.msg import Odometry
from std_msgs.msg import Header
import tf

class RealRobot:
    def __init__(self):
        self.real_robot_publisher = rospy.Publisher("/realrobot", MarkerArray, queue_size=100)
        self.id = 1
        self.markerArrayCube = MarkerArray()
        
        self.Cube = Marker()
        self.Cube.color.r = 0.0
        self.Cube.color.g = 1.0
        self.Cube.color.b = 0.0

        # a needs to be 1, otherwise robot transparent
        self.Cube.color.a = 1.0

        self.Cube.scale.x = 0.1
        self.Cube.scale.y = 0.1
        self.Cube.scale.z = 0.2

        self.Cube.type = 1
        self.Cube.action = 0
        self.Cube.header.frame_id = "odom"
        self.Cube.id = self.id


    def add(self, pos, orien):
        self.pose = Pose(pos, orien)
        self.Cube.pose = self.pose
        

    def send(self):
        self.markerArrayCube.markers.append(self.Cube)
        self.real_robot_publisher.publish(self.markerArrayCube)


class BelievedRobot:
    def __init__(self):
        self.believed_robot_publisher = rospy.Publisher("/believedrobot", MarkerArray, queue_size=100)
        self.id = 2
        self.MarkerArray = MarkerArray()
        self.Cube = Marker()
        self.Cube.color.r = 0.0
        self.Cube.color.g = 0.0
        self.Cube.color.b = 1.0

        # a needs to be 1, otherwise robot transparent
        self.Cube.color.a = 1.0

        self.Cube.scale.x = 0.1
        self.Cube.scale.y = 0.1
        self.Cube.scale.z = 0.2

        # self.Cube.ns = "believed"
        self.Cube.type = 1
        self.Cube.action = 0
        self.Cube.header.frame_id = "odom"
        self.Cube.id = self.id

    def add(self, pos, orien):
        self.pose = Pose(pos, orien)
        self.Cube.pose = self.pose

    def send(self):
        self.MarkerArray.markers.append(self.Cube)
        self.believed_robot_publisher.publish(self.MarkerArray)

def get_real_robot_position(data):
    position = data.pose.pose.position
    orientation = data.pose.pose.orientation

    henry_the_hoover = RealRobot()
    henry_the_hoover.add(position, orientation)
    henry_the_hoover.send()

def get_believed_robot_position(data):
    position = data.pose.pose.position
    orientation = data.pose.pose.orientation

    fake_henry = BelievedRobot()
    fake_henry.add(position, orientation)
    fake_henry.send()
    

if __name__ == "__main__":
    rospy.init_node("robotmarkers", anonymous=True)
    rate = rospy.Rate(1)
    while not rospy.is_shutdown():
        rate.sleep()
        rospy.Subscriber("/base_pose_ground_truth", Odometry, get_real_robot_position)
        rospy.Subscriber("/odom", Odometry, get_believed_robot_position)
