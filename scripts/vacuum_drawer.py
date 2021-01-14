#!/usr/bin/python3
import rospy, cv2, yaml, sys, os, numpy, math
from nav_msgs.msg import Odometry
from std_msgs.msg import Bool

from visualization_msgs.msg import Marker, MarkerArray

class VvacuumMarker:
    def __init__(self, r):
        self.vacuum_marker_publisher = rospy.Publisher("/vacuumed_floor", MarkerArray, queue_size=100)
        self.rate = rospy.Rate(1)
        self.marker_id = 55
        self.markers = []
        self.radius = r


    def add_marker(self, xcoord, ycoord):
        self.marker_id += 1
        new_marker = Marker()
        new_marker.header.frame_id = "map"
        new_marker.scale.x = 0.1
        new_marker.scale.y = 0.1
        # make the cylinder really short to imitate a circle
        new_marker.scale.z = 0.1
        # orange
        new_marker.color.r = 255
        new_marker.color.g = 255
        new_marker.color.b = 255
        new_marker.color.a = 1
        new_marker.type = Marker.CYLINDER
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
        self.vacuum_marker_publisher.publish(self.markerArr)

class InstructionMarker:
    def __init__(self):
        self.instruction_marker_publisher = rospy.Publisher("/room_floor", MarkerArray, queue_size=100)
        self.rate = rospy.Rate(1)
        self.marker_id = 543
        self.markers = []


    def add_marker(self, xcoord, ycoord, x, y):
        self.marker_id += 1
        new_marker = Marker()
        new_marker.header.frame_id = "map"
        new_marker.scale.x = x
        new_marker.scale.y = y
        new_marker.scale.z = 0.05
        new_marker.color.r = 105
        new_marker.color.g = 105
        new_marker.color.b = 105
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

rospy.init_node("vacuum_drawer")

config = []
dirname = os.path.dirname(sys.argv[1])

with open(sys.argv[1],"r") as f:
    config = yaml.safe_load(f)

img = cv2.imread(os.path.join(dirname, config['image']))
gray = cv2.cvtColor(img, cv2.COLOR_BGR2GRAY)
origin = config['origin']
res = config['resolution']
stage_radius = rospy.get_param("vacuum_radius")
vac_radius = int(stage_radius / res)

# @@@@@@@@@@@@@@@@@@@@@@@@@@@@@

def translate_to_pixel(xs,ys):
    return (int((xs - origin[0]) / res), int(img.shape[0] - (ys - origin[1]) / res))

def pixel_to_stage(xp,yp):
    return (int(xp * res + origin[0]), int(((img.shape[0] - yp) * res) + origin[1]))

def compute_distance(x1, y1, x2, y2):
    return math.sqrt((x1 - x2)**2 + (y1 - y2)**2)

def find_centre(bottom_left, top_left, bottom_right, top_right):
    centre_x = (bottom_left[0] + top_left[0] + bottom_right[0] + top_right[0]) / 4
    centre_y = (bottom_left[1] + top_left[1] + bottom_right[1] + top_right[1]) / 4
    return centre_x, centre_y

def is_rectangle(bottom_left, top_left, bottom_right, top_right):
    cx, cy = find_centre(bottom_left, top_left, bottom_right, top_right)

    print("corners: {}, {}, {}, {}\tcentre: [{}, {}]".format(bottom_left, top_left, bottom_right, top_right, cx, cy))
    dd1 = (cx - bottom_left[0])**2 + (cy - bottom_left[1])**2
    dd2 = (cx - top_left[0])**2 + (cy - top_left[1])**2
    dd3 = (cx - bottom_right[0])**2 + (cy - bottom_right[1])**2
    dd4 = (cx - top_right[0])**2 + (cy - top_right[1])**2

    print("d1 = {}\td2 = {}\t d3 = {}\td4 = {}".format(dd1,dd2,dd3,dd4))

    if abs(dd1 - dd2) < 1.55 and  abs(dd1 - dd3) < 1.55 and abs(dd1 - dd2) < 1.55:
        return True
    return False

def get_w_h(bottom_left, top_left, bottom_right, top_right):
    # x axis
    width = abs(bottom_left[0] - bottom_right[0])
    # y axis
    height = abs(bottom_left[1] - top_left[1])
    return width, height

corners = cv2.goodFeaturesToTrack(gray, 150, 0.3, 50)
instructions = rospy.get_param("/instructions")

rooms = []

for instr in instructions:
    pixel_instr = translate_to_pixel(instr[0], instr[1])
    corner_dist = {}
    for corner in corners:
        x, y = corner.ravel()
        corner_dist[tuple((x, y))] = compute_distance(pixel_instr[0], pixel_instr[1], x, y)
    new_dict = dict(sorted(corner_dist.items(), key=lambda item: item[1]))
    keys = list(new_dict.keys())
    room_corners = []
    for i in range(4):
        room_corners.append(pixel_to_stage(keys[i][0], keys[i][1]))
    sorted_corners = sorted(room_corners, key=lambda elem: (elem[0], elem[1]))
    if is_rectangle(sorted_corners[0], sorted_corners[1], sorted_corners[2], sorted_corners[3]):
        cx, cy = find_centre(sorted_corners[0], sorted_corners[1], sorted_corners[2], sorted_corners[3])
        w, h = get_w_h(sorted_corners[0], sorted_corners[1], sorted_corners[2], sorted_corners[3])
        # rooms[tuple((instr[0], instr[1]))] = [cx, cy, w, h]
        rooms.append([cx, cy, w, h])
    print(rooms)
room_floors = InstructionMarker()
for room in rooms:
    room_floors.add_marker(room[0], room[1], room[2], room[3])
# @@@@@@@@@@@@@@@@@@@@@@@@@@@@@

vacuum = False

def handle_vacuum(b):
    global vacuum
    vacuum = b

def handle_bpgt(odom):
    # global img
    if not vacuum:
        return
    x = odom.pose.pose.position.x
    y = odom.pose.pose.position.y
    vacuumed_floor_markers.add_marker(x, y)
    # img = cv2.circle(img, translate_to_pixel(x, y), vac_radius,  (255, 0, 0), -1)

pose_listener = rospy.Subscriber("/base_pose_ground_truth", Odometry, handle_bpgt)

vacuum_listener = rospy.Subscriber("/vacuum", Bool, handle_vacuum)

vacuumed_floor_markers = VvacuumMarker(stage_radius)

rate = rospy.Rate(20)

while not rospy.is_shutdown():
    vacuumed_floor_markers.send()
    room_floors.send()
    # cv2.imshow("vacuuming", img)
    # cv2.waitKey(1)
rospy.spin()