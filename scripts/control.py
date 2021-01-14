#!/usr/bin/python3
import rospy
import math
import numpy as np
from geometry_msgs.msg import Point, Twist, Quaternion
from std_msgs.msg import Float32, Float32MultiArray, Bool
from nav_msgs.msg import Odometry
from sensor_msgs.msg import LaserScan
from tf.transformations import euler_from_quaternion, quaternion_from_euler
from pathplanning import chargers_path

class DriveController:
    def __init__(self, x_msg, y_msg, instr_x, instr_y):
        print("initialising the driver controller")
        self.x_position = None
        self.y_position = None
        self.angle_radians = None

        # path coordinates
        self.x_path_coords = x_msg.data
        self.y_path_coords = y_msg.data

        # instructions in correct order
        self.room_coords_x = list(instr_x.data)
        self.room_coords_y = list(instr_y.data)

        # velocity publisher
        self.cmd_vel_pub = rospy.Publisher('cmd_vel', Twist, queue_size=10)
        # odometry subscriber
        self.odom_sub = rospy.Subscriber("/base_pose_ground_truth", Odometry, self.odometry_callback)
        # obstacle detector subscriber
        self.laser_sub = rospy.Subscriber("base_scan", LaserScan, self.laser_callback, queue_size=1)
        self.vacuum_pub = rospy.Publisher("/vacuum", Bool, queue_size=1)
        self.battery_sub = rospy.Subscriber('/battery', Float32, self.battery_callback)
        self.rate = rospy.Rate(5)

        self.odom = Odometry()
        self.odometry_callback(self.odom)
        self.charger_coords = rospy.get_param("/chargers")
        self.charging = False

        self.current_goal_point = Point() # creating this variable for an ease of reference
        # self.theta_to_goal = None

    def get_theta(self, px, py):
        xs = px - self.x_position
        ys = py - self.y_position
        return math.atan2(ys, xs)

    def get_angle(self):
        return self.angle_radians
    
    def odometry_callback(self, odom_data):
        self.odom = odom_data
        # get the position of the robot
        self.x_position = self.odom.pose.pose.position.x
        self.y_position = self.odom.pose.pose.position.y
        # define a quaternion for rotations
        self.quaterion = (self.odom.pose.pose.orientation.x,
                        self.odom.pose.pose.orientation.y,
                        self.odom.pose.pose.orientation.z,
                        self.odom.pose.pose.orientation.w)
        # get radian value from the quaternion def based on:
        # https://answers.ros.org/question/69754/quaternion-transformations-in-python/
        self.angle_radians = euler_from_quaternion(self.quaterion)[2]

    def laser_callback(self, scan_data):
        self.ranges = scan_data.ranges
        self.left_180 = scan_data.ranges[180]
        self.right_900 = scan_data.ranges[900]
        self.front_540 = scan_data.ranges[540]
        # region_ranges = {"back_left": min(scan_data.ranges[0:180]),
        #                  "left": min(scan_data.ranges[181:360]),
        #                  "front_left": min(scan_data.ranges[361:540]),
        #                  "front_right": min(scan_data.ranges[541:720]),
        #                  "right": min(scan_data.ranges[721:900]),
        #                  "back_right": min(scan_data.ranges[901:1080])}
        self.region_ranges = {"left": min(scan_data.ranges[180:323]),
                         "front_left": min(scan_data.ranges[324:467]),
                         "front": min(scan_data.ranges[468:611]),
                         "front_right": min(scan_data.ranges[612:755]),
                         "right": min(scan_data.ranges[756:900])}
        # self.take_action(region_ranges)
    
    def wall_detected(self):
        if self.region_ranges["front_left"] < 0.22 or self.region_ranges["front"] < 0.2 or self.region_ranges["front_right"] < 0.2:
            return True
        return False
    
    def battery_callback(self, data):
        self.battery_charge = data.data
        # print("battery: {}".format(self.battery_charge))
        if self.battery_charge < 96.0 and self.charging is False:
            self.charging = True
            self.vacuum_pub.publish(False)
    
    def find_charger_path(self):
        print("calculating path to the charger")
        # make sure the robot stops
        hoover_moover = Twist()
        hoover_moover.linear.x = 0
        hoover_moover.angular.z = 0
        self.cmd_vel_pub.publish(hoover_moover)
        self.charger_route, arr = chargers_path(self.x_position, self.y_position)
        self.current_charger = arr[0]
        self.charge_treshold = arr[1]
        # print("charger coords: {}\ttreshold: {}".format(self.current_charger, self.charge_treshold))
        self.drive_to_charge()
    
    def close_to_charger(self):
        if self.check_distance_to_point(self.current_charger[0], self.current_charger[1]) <= self.charge_treshold:
            return True
        return False
    
    def drive_to_charge(self):
        print("driving to charget at {}".format(self.current_charger))
        for i in range(len(self.charger_route)):
            x, y = self.charger_route[i][0], self.charger_route[i][1]
            self.turn_to_point(x, y)
            self.drive_to_point(x, y)
            if self.close_to_charger():
                print("charging from control class")
                hoover_moover = Twist()
                while self.battery_charge < 98:
                    hoover_moover.linear.x = 0
                    hoover_moover.angular.z = 0
                    self.cmd_vel_pub.publish(hoover_moover)
                    self.rate.sleep()
        print("finished charging tour")
        self.charging = False

    def take_action(self, ranges_dict):
        # case 1: no obstacle in anyn section
        if ranges_dict["left"] > 1 and ranges_dict["front_left"] > 1 and ranges_dict["front_right"] > 1 and ranges_dict["right"] > 1:
            # do nothing
            print("case 1: doing nothing")
        # case 2: obstacle directly in front
        elif ranges_dict["left"] > 1 and ranges_dict["front_left"] < 0.3 and ranges_dict["front_right"] < 0.3 and ranges_dict["right"] > 1:
            print("obstacle in front")
        # case 3: obstacle on the left
        elif ranges_dict["left"] < 0.3 and ranges_dict["front_left"] < 0.3 and ranges_dict["front_right"] > 1 and ranges_dict["right"] > 1:
            print("obstacle on the left")
        # case 4: obstacle on the right
        elif ranges_dict["left"] > 1 and ranges_dict["front_left"] > 1 and ranges_dict["front_right"] < 0.3 and ranges_dict["right"] < 0.3:
            print("obstacle on the left")
    def get_closest_point(self):
        # find the next closest point on the path in a starigth line
        min_distance = math.inf
        point_x = None
        point_y = None
        for i in range(len(self.x_path_coords)):
            if self.check_distance_to_point(self.x_path_coords[i], self.y_path_coords[i]) < min_distance:
                point_x = self.x_path_coords[i]
                point_y = self.y_path_coords[i]

        print("closest point found: [{}, {}]".format(point_x, point_y))
        return point_x, point_y

    # function computes the smallest turn the robot has to do in order to face the goal
    def  normalize_angle(self, angl):
        d_angl = math.degrees(angl)
        if d_angl > 180:
            return math.radians(-(180 - (d_angl - 180)))
        elif d_angl < -180:
            return math.radians(180 + (d_angl + 180))
        return angl

    def close_to_instruction(self):
        if len(self.room_coords_x) == 0:
            return False
        if self.check_distance_to_point(self.room_coords_x[0], self.room_coords_y[0]) <= 0.05 and self.charging is False:
            return True
        return False

    # TODO: establish turn direction to minimise time
    def turn_to_point(self, px, py):
        print("turning")
        # define a Twist object to move the robot and give it angular velocity
        hoover_moover = Twist()
        # hoover_moover.angular.z = 0.2
        speed = 0.9
        # get the angle difference (theta) and normalize it to ioptimise turning
        angle_difference = self.normalize_angle(self.get_angle() - self.get_theta(px, py))

        # turn until the angle difference is acceptable
        while (abs(angle_difference) > 0.03):
            angle_difference = self.normalize_angle(self.get_theta(px, py) - self.get_angle())
            # set and publish angular velocity
            hoover_moover.angular.x = 0
            hoover_moover.angular.y = 0
            hoover_moover.angular.z = angle_difference * speed
            self.cmd_vel_pub.publish(hoover_moover)
            self.rate.sleep()

        # decrease velocity to 0 to stop the robot
        hoover_moover.angular.z = 0
        hoover_moover.linear.x = 0
        # print("stop turning")
        self.cmd_vel_pub.publish(hoover_moover)

    def drive_to_point(self, px, py):
        print("driving")
        # compute distance to goal and compare it with a threshold
        distance_to_goal = self.check_distance_to_point(px, py)
        # print("Current point [{}, {}] is {} units away".format(self.current_goal_point.x, self.current_goal_point.y, distance_to_goal))
        # just a constant for calculations
        hoover_moover = Twist()

        # defining a time unit for driving
        timestamp_1 = rospy.Time.now().to_sec()
        timestamp_2 = rospy.Time.now().to_sec()

        speed = 0.2

        # drive to point
        while (((timestamp_2 - timestamp_1) * speed) < distance_to_goal):
            timestamp_2 = rospy.Time.now().to_sec()
            hoover_moover.linear.x = speed
            self.cmd_vel_pub.publish(hoover_moover)            

        # decrease velocity to 0 to stop the robot
        hoover_moover.linear.x = 0
        hoover_moover.angular.z = 0
        print("stop moving")
        self.cmd_vel_pub.publish(hoover_moover)

    def check_distance_to_point(self, px, py):
        distance = math.sqrt((px- self.x_position)**2 + (py - self.y_position)**2)
        return distance

    def follow_path(self):
        self.rate.sleep()
        self.rate.sleep()
        self.rate.sleep()
        self.rate.sleep()
        self.rate.sleep()
        print("starting to follow the path")
        # call the driving method for every point on the path
        for i in range(len(self.x_path_coords)):
            # update current final point
            self.current_goal_point.x = self.x_path_coords[i]
            self.current_goal_point.y = self.y_path_coords[i]
            # print("driving from ({}, {}) to: ({}, {})".format(self.x_position, self.y_position, self.current_goal_point.x, self.current_goal_point.y))
            # drive to a point only if robot is not already there
            if self.x_position != self.current_goal_point.x and self.y_position != self.current_goal_point.y:
                if self.charging is True:
                    self.find_charger_path()
                # turn to a point
                self.turn_to_point(self.current_goal_point.x, self.current_goal_point.y)
                # drive to a point
                self.drive_to_point(self.current_goal_point.x, self.current_goal_point.y)
            if self.close_to_instruction():
                print("vacuuming")
                # print("front sensor: {}\tleft sensor: {}\tright sensor: {}".format(self.front_540, self.left_180, self.right_900))
                self.vacuum_room()
        print("tour finished")

    def vacuum_room(self):
        vacuuming_happening = True
        self.vacuum_pub.publish(vacuuming_happening)
        print("Vacuuming room at [{}, {}]".format(self.current_goal_point.x, self.current_goal_point.y))
        hoover_moover = Twist()
        some_vel = 0
        loop_rate = rospy.Rate(10)
        # when hit the wall do:
        # save start point, trace wall until start point is met or wall ends
        
        while self.charging is False:
            if self.wall_detected():
                print("wall detected")
                self.vacuum_pub.publish(False)
                break
            some_vel += 0.01
            hoover_moover.linear.x = some_vel
            hoover_moover.angular.z = 1.2
            self.cmd_vel_pub.publish(hoover_moover)
            loop_rate.sleep()
        
        if self.charging is False:
        # get rid of the room coords to indicatee the room has been finished
            self.room_coords_x.pop(0)
            self.room_coords_y.pop(0)
        hoover_moover.linear.x = 0
        hoover_moover.angular.z = 0
        self.cmd_vel_pub.publish(hoover_moover)
        vacuuming_happening = False
        self.vacuum_pub.publish(vacuuming_happening)

        # trace the wall once

if __name__ == "__main__":
    try:
        rospy.init_node("driving", anonymous=True)
        # the node waits to get messages from path planning
        x_path_coords_sub = rospy.wait_for_message("/best_path_x_coords", Float32MultiArray)
        y_path_coords_sub = rospy.wait_for_message("/best_path_y_coords", Float32MultiArray)

        x_instr_coords = rospy.wait_for_message("/instr_x_coords", Float32MultiArray)
        y_instr_coords = rospy.wait_for_message("/instr_y_coords", Float32MultiArray)

        # creating the driving controller
        driver = DriveController(x_path_coords_sub, y_path_coords_sub, x_instr_coords, y_instr_coords)
        driver.follow_path()

        rospy.spin()
    except rospy.ROSInterruptException:
        pass