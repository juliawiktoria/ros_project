#!/usr/bin/python3
import rospy
from std_msgs.msg import Float32MultiArray
from visualization_msgs.msg import Marker, MarkerArray
from geometry_msgs.msg import Point, Pose
from nav_msgs.msg import OccupancyGrid, Odometry
import math
import convert_coords as convo
import itertools
from astar_planner import AstarPathPlanning
import json
import time
import numpy

# @@@@@@@@@@@@@@@@@@@@@@@@@@@@@@

class MyMapp:
    # initialises a ROS node and creates a subscriber to the topic with the map
    def __init__(self):
        # print("map init")
        self.grid_msg = rospy.wait_for_message("/map", OccupancyGrid)
        self.map_callback()
        
    # getting all the map data from the topic
    def map_callback(self):
        data = self.grid_msg
        # print("map callback")
        self.raw_map = OccupancyGrid()
        self.raw_map.data = data.data
        self.raw_map.info = data.info
        self.map_height = self.raw_map.info.height
        self.map_width = self.raw_map.info.width
        self.map_res = self.raw_map.info.resolution
        self.origin_x = self.raw_map.info.origin.position.x
        self.origin_y = self.raw_map.info.origin.position.y
    
        self.buffed_map = [[0 for x in range(self.map_width)] for y in range(self.map_height)]
        self.grid_map = numpy.ndarray((self.map_width, self.map_height))

        # translating the map based on LINK
        for i in range(len(self.grid_map)):
            self.grid_map[(i % self.map_width)][int((i / self.map_width))] = self.raw_map.data[i]
        
        # "buffing" around obstacles in order to improve driving (not driving too close to the obstacle)
        # bufing rate of 6 was chosen by trial and error testing
        how_close = 6
        i = 0
        for row in range(0, self.map_height):
            for column in range(0, self.map_width):
                if (self.raw_map.data[i] != 0):
                    for yy in range(-how_close, how_close):
                        for xx in range(-how_close, how_close):
                            if row + yy < 0 or column + xx < 0 or row + yy >= self.map_height or column + xx >= self.map_width:
                                continue
                            # sets the value to 1 as in occupied
                            self.buffed_map[row + yy][column + xx] = 1
                i += 1

# @@@@@@@@@@@@@@@@@@@@@@@@@@@@@@

class PathMarkers(object):
    def __init__(self):
        self.marker_objectpublisher= rospy.Publisher("/calculatedPath", MarkerArray,  queue_size=100)
        self.rate = rospy.Rate(1)
        self.markerid=16
        self.markers = []

    def add_marker(self,x,y,r,g,b):
        self.markerid+=1
        marker_obj=Marker()
        marker_obj.header.frame_id="map"
        marker_obj.scale.x=0.1
        marker_obj.scale.y=0.1
        marker_obj.scale.z=0.1
        marker_obj.color.r=r
        marker_obj.color.g=g
        marker_obj.color.b=b
        marker_obj.color.a=1.0
        marker_obj.type=Marker.CUBE
        marker_obj.action=Marker.ADD
        marker_obj.pose.position.x =x
        marker_obj.pose.position.y =y
        marker_obj.pose.orientation.w=1.0
        marker_obj.id = self.markerid #otherwise error
        # print(marker_obj,"MY MARKEWR")
        self.markers.append(marker_obj)


    def publish(self):
        self.markerArray = MarkerArray()
        for marker in self.markers:
            self.markerArray.markers.append(marker)
        self.marker_objectpublisher.publish(self.markerArray)


def start(computed_path, orx, ory, res, charging=False):
    if charging is False:
        r,g,b = 5,1,1
    else:
        r,g,b=153,0,153
    # print("time for drawing")
    marker = PathMarkers()
    for p in computed_path:
        point = convo.grid_to_stage(p, orx, ory, res)
        # print("point coords: {}".format(point))
        marker.add_marker(point[0], point[1], r, g, b)
    # print("points converted to stage")
    while not rospy.is_shutdown():
        rate = rospy.Rate(1)
        rate.sleep()
        marker.publish()

    rospy.spin() #go into infinite loop until receive a shutdown sygnam (ctr-c)

def vacuum_everything():
    print("Working with no instructions haven't been implemented!!!! I am tired")
    return

def get_grid():
    # print("getting grid")
    map_img = MyMapp()
    print(type(map_img))
    # map_sub = rospy.Subscriber("/map", OccupancyGrid, map_img.map_callback)
    grid = map_img.buffed_map
    origin_x = map_img.origin_x
    origin_y = map_img.origin_y
    resolution = map_img.raw_map.info.resolution
    # print("got grriid: {}, {}, {}".format(origin_x, origin_y, resolution))
    # return grid
    return grid, origin_x, origin_y, resolution

def get_destination_pairs(arr):
    # returns a list of all possible pairs of points without repetitions
    return list(itertools.combinations(arr, 2))

def create_tours(arr):
    # detach the robot because it is always the starting point
    robot = arr.pop(0)
    all_permutations = list(itertools.permutations(arr))
    robot_permutations = []
    # make sure all tours start with robot's origin
    for elem in all_permutations:
        new_list = [robot]
        new_list.extend(elem)
        robot_permutations.append(tuple(new_list))
    return robot_permutations
# 
def evaluate_tour(arr, pair_distance_dict):
    distance = 0
    for i in range(len(arr) - 1):
        # account for connection both ways (a->b = b->a)
        if (arr[i],arr[i+1]) in pair_distance_dict.keys():
            distance += pair_distance_dict[(arr[i],arr[i+1])]
        else:
            distance += pair_distance_dict[(arr[i + 1],arr[i])]
    return distance

def combine_final_tour(arr, partial_tours_dict):
    final_tour = []
    for i in range(len(arr) - 1):
        if (arr[i], arr[i+1]) in partial_tours_dict.keys():
            final_tour.extend(partial_tours_dict[(arr[i], arr[i + 1])])
        elif (arr[i + 1], arr[i]) in partial_tours_dict.keys():
            reversed_path = partial_tours_dict[(arr[i + 1], arr[i])][::-1]
            final_tour.extend(reversed_path)
    return final_tour

def chargers_path(current_x, current_y):
    print("trying to calc charger path")
    grid_map, ox, oy, grid_res = get_grid()
    charger_dict = {}
    chargers_stage = rospy.get_param("/chargers")
    print("chargers: {}".format(chargers_stage))
    chargers_grid = [convo.stage_to_grid(elem, ox, oy, grid_res) for elem in chargers_stage]
    for i in range(len(chargers_stage)):
        radius = chargers_stage[i].pop()
        charger_dict[tuple(chargers_grid[i])] = tuple((chargers_stage[i], radius))
    print(charger_dict)
    pairs = []
    start_grid = convo.stage_to_grid(tuple((current_x, current_y)), ox, oy, grid_res)
    for elem in chargers_grid:
        pairs.append([tuple(start_grid), tuple(elem)])

    # run astar for every pair and note results in corresponding dictionaries
    pair_distance = {}
    pair_tour = {}
    count = 1
    for each in pairs:
        print("a start for pair #{} {}".format(count,each))
        path_planner_obj = AstarPathPlanning(grid_map)
        x, astar_result = path_planner_obj.a_star_algorithm(each[0], each[1])
        # add result if it exists
        if astar_result != -1:
            # print("each: {}, type: {}".format(each, type(each)))
            pair_tour[tuple(each)] = astar_result
            pair_distance[tuple(each)] = len(astar_result)
        if astar_result == -1:
            # if there is no path make the dist for this pair to be infinity for ease of future calculations
            pair_distance[tuple(each)] = math.inf
        count += 1
    print(pair_distance)
    # closest charger, returns the key - list of grid coordinates [(start), (charger)]
    closest_charger = min(pair_distance, key=pair_distance.get)
    # print("closest charger: {}".format(closest_charger))
    path_to_charger = pair_tour[closest_charger]
    print("path to charger: {}".format(path_to_charger))
    # path to the charger and back
    smoother = smooth_path(path_to_charger)
    print("smoother path: {}".format(smoother))
    smoother.extend(reversed(smoother[:-1]))
    # print("smoother path: {}".format(smoother))
    # start(smoother, ox, oy, grid_res)
    final_pth = [convo.grid_to_stage(elem, ox, oy, grid_res) for elem in smoother]
    # print("final charger path: {}".format(final_pth))
    print(charger_dict[closest_charger[1]])
    return final_pth, charger_dict[closest_charger[1]]


def publish_best_path(best_path_coords, orx, ory, res, instr_tour):
    print("trying to publish path coords")
    
    x_publisher = rospy.Publisher("/best_path_x_coords", Float32MultiArray, queue_size=10, latch=True)
    y_publisher = rospy.Publisher("/best_path_y_coords", Float32MultiArray, queue_size=10, latch=True)

    x_instructions_pub = rospy.Publisher("/instr_x_coords", Float32MultiArray, queue_size=10, latch=True)
    y_instructions_pub = rospy.Publisher("/instr_y_coords", Float32MultiArray, queue_size=10, latch=True)

    msg_x = Float32MultiArray()
    msg_y = Float32MultiArray()

    instr_x = Float32MultiArray()
    instr_y = Float32MultiArray()

    try:
        new_path = smooth_path(best_path_coords)
        # print("new path len: {}".format(len(new_path)))
    except IndexError:
        print("eereo")
        new_path = best_path_coords

    for point in new_path:
        new_point = convo.grid_to_stage(point, orx, ory, res)
        msg_x.data.append(round(new_point[0], 2))
        msg_y.data.append(round(new_point[1], 2))

    for point in instr_tour:
        instr_x.data.append(point[0])
        instr_y.data.append(point[1])

    x_publisher.publish(msg_x)
    y_publisher.publish(msg_y)

    x_instructions_pub.publish(instr_x)   
    y_instructions_pub.publish(instr_y)   

def on_the_same_line(slope, point_a, point_b):
    if abs((point_b[1] - point_a[1]) - slope *  (point_b[0] - point_a[0])) <= 0.05:
        return True
    return False

def compute_slope(point_a, point_b):
    if (point_b[0] - point_a[0]) != 0:
        return (point_b[1] - point_a[1]) / (point_b[0] - point_a[0])
    else:
        return 0

def smooth_path(arr):
    new_array = []
    # first two points create a default segment
    current_start = arr[0]
    current_end = arr[1]
    current_slope = compute_slope(current_start, current_end)
    new_array.append(current_start)

    # weird loop in order to have control over indices
    for i in range(2, len(arr) - 1, 2):
        if on_the_same_line(current_slope, current_start, arr[i]):
            current_end = arr[i]
            if on_the_same_line(current_slope, current_start, arr[i + 1]):
                current_end = arr[i + 1]
            else:
                new_array.append(current_end)
                current_start = arr[i]
                current_end = arr[i + 1]
                current_slope = compute_slope(current_start, current_end)
        else:
            new_array.append(current_end)
            current_start = current_end
            current_end = arr[i]
            current_slope = compute_slope(current_start, current_end)
    new_array.append(arr[-1])
    return new_array


if __name__ == "__main__":
    rospy.init_node("pathplanning", anonymous=True)
    try:
        start_time = time.time()
        print("pathplanning started")
        # get map and some of its parameters for performing the path planning
        grid, or_x, or_y, reso = get_grid()
        order_in_stage_coords = {}
        # get one msg from odom to set the robot's spawn point
        spawn_sub = rospy.wait_for_message("/odom", Odometry)
        # ignore error, works just fine
        spawn = tuple((spawn_sub.pose.pose.position.x, spawn_sub.pose.pose.position.y))

        # # convert to grid coords for future reference
        # grid_spawn_coords = tuple(convo.stage_to_grid(spawn, or_x, or_y, reso))

        # check if there are vacuuming instructions
        if rospy.has_param("/instructions"):
            instruction_points = rospy.get_param("/instructions")
        else:
            # if no instructions, vacuum everything
            vacuum_everything()

        # convert instructions given in stage coordinates into grid coordinates using custom made convertion functions
        grid_instructions = []

        # check if robot starts in a room that also needs to be vacuumed
        if list(spawn) in instruction_points:
            instruction_points.insert(0, instruction_points.pop(instruction_points.index(list(spawn))))
        else:
            # append robot grid coords so it is in the beginning of the list
            # grid_instructions.append(tuple(convo.stage_to_grid(spawn, or_x, or_y, reso)))
            instruction_points.insert(0, spawn)

        for point in instruction_points:
            grid_point = tuple(convo.stage_to_grid(point, or_x, or_y, reso))
            grid_instructions.append(grid_point)
            order_in_stage_coords[grid_point] = tuple(point)
        print("coordinates were converted to grid ones")
        print(order_in_stage_coords)
            
        # create all possible pair within all instruction points and the spawn point
        destination_pairs = get_destination_pairs(grid_instructions)
        print("pairs created: #{}".format(len(destination_pairs)))

        # run astar for every pair and note results in corresponding dictionaries
        pair_distance = {}
        pair_tour = {}
        count = 1
        for each in destination_pairs:
            print("a start for pair #{} {}".format(count,each))
            path_planner_obj = AstarPathPlanning(grid)
            x, astar_result = path_planner_obj.a_star_algorithm(each[0], each[1])
            # add result if it exists
            if astar_result != -1:
                # print("each: {}, type: {}".format(each, type(each)))
                pair_tour[tuple(each)] = astar_result
                pair_distance[tuple(each)] = len(astar_result)
            if astar_result == -1:
                # if there is no path make the dist for this pair to be infinity for ease of future calculations
                pair_distance[tuple(each)] = math.inf
            count += 1
        print("astar completed")

        # generate all possible paths going through goals
        possible_tours = create_tours(grid_instructions)
        print("tours generated")
        tour_distance = {}
        for each in possible_tours:
            tour_distance[each] = evaluate_tour(each, pair_distance)
        print("tour distances calculated")

        # choose the best one - tour with the lowest distance value
        best_tour = min(tour_distance, key=tour_distance.get)
        print("shortest path found: {}".format(best_tour))
        stage_tour_order = []
        for i in range(len(best_tour)):
            stage_tour_order.append(order_in_stage_coords[best_tour[i]])
        print(stage_tour_order)
        final_path = combine_final_tour(best_tour, pair_tour)
        # print("final path combined: {}".format(final_path))
        end_time = time.time()

        # print("elapsed time: {}".format(end_time - start_time))

        publish_best_path(final_path, or_x, or_y, reso, stage_tour_order)

        start(final_path, or_x, or_y, reso, stage_tour_order)
    except rospy.ROSInterruptException:
        pass

