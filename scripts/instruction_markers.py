#!/usr/bin/python3
import rospy
from visualization_msgs.msg import Marker, MarkerArray

# class responsible for gettting goal instructions and publishing them to Rviz as markers
class InstructionMarker:
    def __init__(self, id):
        self.instruction_marker_publisher = rospy.Publisher("/instructions", MarkerArray, queue_size=100)
        self.rate = rospy.Rate(1)
        self.marker_id = id
        self.markers = []


    def add_marker(self, xcoord, ycoord, x, y, z, r, g, b):
        self.marker_id += 1
        new_marker = Marker()
        new_marker.header.frame_id = "map"
        new_marker.scale.x = x
        new_marker.scale.y = y
        new_marker.scale.z = z
        new_marker.color.r = r
        new_marker.color.g = g
        new_marker.color.b = b
        new_marker.color.a = 1
        new_marker.type = Marker.CUBE
        new_marker.action = Marker.ADD
        new_marker.pose.position.x = xcoord
        new_marker.pose.position.y = ycoord
        new_marker.pose.orientation.w = 1.0
        new_marker.id = self.marker_id
        self.markers.append(new_marker)


    def send(self):
        self.markerArr = MarkerArray()
        for point in self.markers:
            self.markerArr.markers.append(point)
        self.instruction_marker_publisher.publish(self.markerArr)

rospy.init_node("instructionmarkers", anonymous=True)

# check if instructions exist
if rospy.has_param("/instructions"):
    pointers = rospy.get_param("/instructions")
else:
    pointers = [ [ 1, 1 ] ]

charging_points = rospy.get_param("/chargers")

instructions = InstructionMarker(44)
chargers = InstructionMarker(777)

for each in pointers:
    instructions.add_marker(each[0], each[1], 0.1, 0.1, 1.0, 1, 2, 3)
for each in charging_points:
    chargers.add_marker(each[0], each[1], 0.1, 0.1, 1.0, 0, 0, 0)

while not rospy.is_shutdown():
    rate = rospy.Rate(1)
    rate.sleep()
    instructions.send()
    chargers.send()
rospy.spin()