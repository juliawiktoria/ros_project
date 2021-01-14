#!/usr/bin/python3

# functions responsible for coordinate transformations

# convert stage coordinated to coordinates on the occupancy grid
def stage_to_grid(coordinates, origin_x, origin_y, res):
    x_coord = coordinates[0]
    y_coord = coordinates[1]

    result = [int((x_coord - origin_x) / res), int((y_coord - origin_y) / res)]

    # if(result[0] % 2 == 1):
    #     result[0] -= 1
    
    return result

# converts occupancy grid coordinates to stage coordinates
def grid_to_stage(coordinates, origin_x, origin_y, res):
    x_coord = coordinates[1]
    y_coord = coordinates[0]

    return [origin_x + x_coord * res, origin_y + y_coord * res]
