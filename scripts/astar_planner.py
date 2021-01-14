#!/usr/bin/python3
import rospy

class AstarPathPlanning:
    def __init__(self, grid):
        self.grid_map = grid

    # compute a heuristic value for each entry in the occupancy grid
    def calculate_heuristic(self):
        rows_no = len(self.grid_map)
        columns_no = len(self.grid_map[0])

        # initialise list with 0's to hold heuristic values so that appending is easier
        self.heuristic_list = [[0 for x_coord in range(columns_no)] for y in range(rows_no)]
        for i in range(rows_no):
            # compute difference in X coords
            x_difference = abs(i - self.goal_point[0])
            for j in range(columns_no):
                # compute difference in Y coords
                y_difference = abs(j - self.goal_point[1])
                # use euclidean distance to compute heuristic
                self.heuristic_list[i][j] = int(abs(x_difference - y_difference) + min(x_difference, y_difference) * 2)
            

    def a_star_algorithm(self, start_coords, goal_coords):

        start = [start_coords[1], start_coords[0]]
        goal = [goal_coords[1], goal_coords[0]]
        
        # Calculate the Heuristic for the map using the goal point coordinates
        self.goal_point = goal
        self.calculate_heuristic()

        # definition of coordinates change for each point in the grid
        neighbour_coords = [[-1, 0], [0, -1], [1, 0], [0, 1], [-1, -1], [1, -1], [-1, 1], [1, 1]]
        # name of the direction the neighbour is in; order is corresponding
        neighbour_name = ['up ', 'left ', 'down ', 'right ', 'upper_left ', 'lower_left', 'upper_right', 'lower_right']

        # a list defining if a certain node in the occupancy gris is closed (0 for open, 1 for closed)
        closed_nodes = [[0 for col in range(len(self.grid_map[0]))] for row in range(len(self.grid_map))]
        
        path_markings = [['  ' for _ in range(len(self.grid_map[0]))] for _ in range(len(self.grid_map))]
        
        # closing the start node that has been passed as a parameter
        closed_nodes[start[0]][start[1]] = 1

        # a list flaging which points have been expanded (-1 for not expanded)
        explore = [[-1 for col in range(len(self.grid_map[0]))] for row in range(len(self.grid_map))]
        neighbour_tracking = [[-1 for _ in range(len(self.grid_map[0]))] for _ in range(len(self.grid_map))]

        # the first node; g value is 0 because it is the start node
        cost = 1
        x_coord = start[0]
        y_coord = start[1]
        g_value = 0
        # f_value is computed using current g_value and heuristic score for inspected node
        f_value = g_value + self.heuristic_list[x_coord][y_coord]
        
        # adding a node to a list tracking open nodes together with their parameters
        open_nodes = [[f_value, g_value, x_coord, y_coord]]

        path_found = False  # flag that is set when search is complete
        cannot_explore_more = False  # flag set if we can't find expand
        # keeping count of the number of iterations to make sure the program doesnt loop
        counter = 0
        
        while not path_found and not cannot_explore_more:
            # terminate if there are no open nodes to explore
            if len(open_nodes) == 0:
                cannot_explore_more = True
                
            else:
                # sort open nodes in order to find one with the lowest f value, and make it the next one to be explored
                open_nodes.sort()
                open_nodes.reverse()
                next_node = open_nodes.pop()
                
                # get info about the node
                g_value = next_node[1]
                x_coord = next_node[2]
                y_coord = next_node[3]

                explore[x_coord][y_coord] = counter
                counter += 1

                # break the loop of too many iterations have passed
                if counter > 184656:
                    return 0, -1

                # checking if the goal node was reached
                if x_coord == goal[0] and y_coord == goal[1]:
                    # setting this to True breaks the loop
                    path_found = True
                
                # doing if the goal was not found
                else:
                    # create node neighbours
                    for i in range(len(neighbour_coords)):
                        # using the coord change definition to find neighbours
                        x_new = x_coord + neighbour_coords[i][0]
                        y_new = y_coord + neighbour_coords[i][1]
                        # checking if created neighbour coords do not exceed the bound of the grid
                        if len(self.grid_map) > x_new >= 0 and len(self.grid_map[0]) > y_new >= 0:
                            # check if the node is not closed and if there is no obstacle
                            if closed_nodes[x_new][y_new] == 0 and self.grid_map[x_new][y_new] == 0:
                                # compute new g and f value for the new node and add it to the open list
                                g_new = g_value + cost
                                f_value = g_new + self.heuristic_list[x_new][y_new]
                                open_nodes.append([f_value, g_new, x_new, y_new])
                                closed_nodes[x_new][y_new] = 1
                                neighbour_tracking[x_new][y_new] = i
        
        # after the goal was reached backtracking needs to be done to form the path
        curr_x = goal[0]
        curr_y = goal[1]
        path_markings[curr_x][curr_y] = '@ '
        final_path_coordinates = []
        final_path_coordinates.append(goal)
        # backtrack until start node is reached
        while curr_x != start[0] or curr_y != start[1]:
            # compute previous point based on the neighbouthood tracking
            prev_x = curr_x - neighbour_coords[neighbour_tracking[curr_x][curr_y]][0]
            prev_y = curr_y - neighbour_coords[neighbour_tracking[curr_x][curr_y]][1]
            path_markings[prev_x][prev_y] = neighbour_name[neighbour_tracking[curr_x][curr_y]]
            # append path coordinates
            final_path_coordinates.append((curr_x, curr_y))
            # move to the previous node that has just been appended to the path to find its precedessor
            curr_x = prev_x
            curr_y = prev_y
        
        # reversing the list so the path is presented in the proper order
        final_path_coordinates.reverse()

        # return the path but without the start node in it so it can be easily concatenated
        return start, final_path_coordinates[:-1]