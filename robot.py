import numpy as np
import time
import random
import sys

forward = [[-1,  0], # go up
           [ 0, -1], # go left
           [ 1,  0], # go down
           [ 0,  1]] # go right
forward_name = ['up', 'left', 'down', 'right']

# action has 3 values: right turn, no turn, left turn
action = [-1, 0, 1]
action_name = ['R', '#', 'L']

dir_names   = ['up', 'left', 'down', 'right']
angle_val = [-90, 0, 90]
class Robot(object):
    def __init__(self, maze_dim):
        '''Use the initialization function to set up attributes that your robot
        will use to learn and navigate the maze. Some initial attributes are
        provided based on common information, including the size of the maze
        the robot is placed in.'''
        

        self.heading = 'up'
        self.maze_dim = maze_dim
        self.no_of_rows = maze_dim
        self.no_of_cols = maze_dim
        self.location = [self.no_of_rows-1, 0]


    def isGoal(self, location):
        row_no, col_no = location
        return (self.no_of_rows/2-1) <= row_no \
                and row_no <= (self.no_of_rows/2) and \
               (self.no_of_cols/2-1) <= col_no \
               and col_no <= (self.no_of_cols/2)

    def move(self, steering, distance):

        # make a new copy
        curr_pos  = dir_names.index(self.heading)
        act_index = angle_val.index(steering)
        dir_index = (curr_pos+action[act_index])%len(dir_names)
        self.heading = dir_names[dir_index]

        delta     = forward[dir_index] 
        self.location  = [ self.location[i]+delta[i]*distance for i in range(2) ]
        #return    (direction, location)


    def next_move(self, sensors):
        '''
        Use this function to determine the next move the robot should make,
        based on the input from the sensors after its previous move. Sensor
        inputs are a list of three distances from the robot's left, front, and
        right-facing sensors, in that order.

        Outputs should be a tuple of two values. The first value indicates
        robot rotation (if any), as a number: 0 for no rotation, +90 for a
        90-degree rotation clockwise, and -90 for a 90-degree rotation
        counterclockwise. Other values will result in no rotation. The second
        value indicates robot movement, and the robot will attempt to move the
        number of indicated squares: a positive number indicates forwards
        movement, while a negative number indicates backwards movement. The
        robot may move a maximum of three units per turn. Any excess movement
        is ignored.

        If the robot wants to end a run (e.g. during the first training run in
        the maze) then returing the tuple ('Reset', 'Reset') will indicate to
        the tester to end the run and return the robot to the start.
        '''
        time.sleep(0.5)
        sensors_array = np.array(sensors)
        #looks for possible indexes for taking random turn
        sensors_array = np.where(sensors_array>0)

        sensors_array = sensors_array[0]  #for getting first element that is size of that array

        if len(sensors_array) >0:

            #Assigns random value to the turn for initial exploration
            rand_index = random.choice(sensors_array)

            rotation = angle_val[rand_index]
            movement = 1
        else:
            #if that was the dead end case

            rotation = 90
            movement = 1
        #print self.isGoal(self.location)
        if self.isGoal(self.location):
            print 'Reached Goal'
            sys.exit()
        self.move(rotation, movement)
        print 'rotation ',rotation,' movement ', movement
        print 'heading ',self.heading,' location ', self.location
        return rotation, movement






















