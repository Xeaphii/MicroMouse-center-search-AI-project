import numpy as np
import time
import sys
from random import random
from bisect import bisect

forward = [[-1,  0], # go up
           [ 0, -1], # go left
           [ 1,  0], # go down
           [ 0,  1]] # go right
forward_name = ['up', 'left', 'down', 'right']

# action has 3 values: right turn, no turn, left turn
action = [1, 0, -1]
action_name = ['L', '#', 'R']

dir_names   = ['up', 'left', 'down', 'right']
angle_val = [-90, 0,90]

dir_index_ar = [0,3,2,1] # for up right down left wall orientation
class Robot(object):
    def __init__(self, maze_dim):
        '''Use the initialization function to set up attributes that your robot
        will use to learn and navigate the maze. Some initial attributes are
        provided based on common information, including the size of the maze
        the robot is placed in.'''
        

        self.heading     = 'up'
        self.maze_dim    = maze_dim
        self.no_of_rows  = maze_dim
        self.no_of_cols  = maze_dim
        self.init_val    = 0
        self.def_heu_val = -1
        self.init_loc    = [self.no_of_rows-1, 0]
        self.location    = self.init_loc 

        self.count_grid  = [[0 for row in range(self.no_of_rows)] 
                  for col in range(self.no_of_cols)]

        self.mapped_grid = [[self.init_val for row in range(self.no_of_rows)] 
                  for col in range(self.no_of_cols)]

        self.deadend_grid = [[0 for row in range(self.no_of_rows)] 
                  for col in range(self.no_of_cols)]

        self.heuristics = [[self.def_heu_val for row in range(self.no_of_rows)]
                 for col in range(self.no_of_cols)]

        x,y = self.location
        #self.count_grid[x][y] = 1
        self.is_previous_loc_deadend = False
        self.self_exploration_val    = 10.
        self.epsilon_val = 0.0000001
        self.count_steps = 0
        self.size_of_goal= 2
        self.no_of_dim   = 2


    def test_coverage():
        return 
    def isGoal(self, location):
        row_no, col_no = location
        return (self.no_of_rows/2-1) <= row_no \
                and row_no <= (self.no_of_rows/2) and \
               (self.no_of_cols/2-1) <= col_no \
               and col_no <= (self.no_of_cols/2)

    def is_all_space_explored(self):
        for i in range(len(self.count_grid)):
            for j in range(len(self.count_grid[0])):
                if self.count_grid[i][j] ==0:
                    return False
        return True

    def move(self, steering, distance):

        # make a new copy
        curr_pos  = dir_names.index(self.heading)
        act_index = angle_val.index(steering)
        dir_index = (curr_pos+action[act_index])%len(dir_names)
        self.heading = dir_names[dir_index]

        delta     = forward[dir_index] 
        self.location  = [ self.location[i]+delta[i]*distance for i in range(2) ]
        #return    (direction, location)

    def print_list(self,input_list):
        for inner_list in input_list:
            print inner_list,'\n'
        print '\n \n'

    def update_counter(self):
        #Updating location count     
        x,y = self.location
        self.count_grid[x][y]  += 1

    def update_deadends(self):
        #Updating location count     
        x,y = self.location
        self.deadend_grid[x][y]  += 1

    def update_heuristics(self,location,h_value):
        #Updating location count     
        x,y = location
        self.heuristics[x][y] = h_value

    def update_mapping(self,value):
        #Updating location count     
        # - up = 1 = 2^0 = 2^Direction.N.value 
        # - right  = 2 = 2^1 = 2^Direction.E.value
        # - down = 3 = 2^2 = 2^Direction.S.value
        # - left  = 4 = 2^3 = 2^Direction.W.value

        x,y = self.location
        self.mapped_grid[x][y] = value

    def weighted_choice(self,choices):
        values, weights = zip(*choices)
        total = 0
        cum_weights = []
        for w in weights:
            total += w
            cum_weights.append(total)
        x = random() * total
        i = bisect(cum_weights, x)
        return values[i]

    def simulate_move(self, steering, distance):

        # make a new copy
        curr_pos  = dir_names.index(self.heading)
        act_index = angle_val.index(steering)
        dir_index = (curr_pos+action[act_index])%len(dir_names)

        delta     = forward[dir_index] 
        return [ self.location[i]+delta[i]*distance for i in range(2) ]

    def get_corrected_orientation(self, steering):

        # make a new copy
        curr_pos  = dir_names.index(self.heading)
        act_index = steering
        dir_index = (curr_pos+action[act_index])%len(dir_names)
        return dir_index_ar[dir_index]


    def robot_exploration(self,sensors):

        #For updating mapping of space start
        back_index = [0,1,2,3]
        value = 0
        for idx,sensor_reading in enumerate(sensors):
            cor_index = self.get_corrected_orientation(idx)
            back_index.remove(cor_index)
            if sensor_reading>0:
                value += 2**cor_index

               
        


        #For updating mapping of space end

        #time.sleep(0.1)
        sensors_array = np.array(sensors)
        
        #looks for possible indexes for taking random turn
        sensors_array = np.where(sensors_array>0)

        sensors_array = sensors_array[0]  #for getting first element that is size of that array
        self.update_counter()        

        if len(sensors_array) >0:

            if self.init_loc != self.location and\
                 self.is_previous_loc_deadend == False: #Cheque for initial location
                value += 2**back_index[0]   #Value for the back index

            #Assigns random value to the turn for initial exploration
            #rand_index = random.choice(sensors_array)

            weighted_array = []
            for sensors_item in sensors_array:
                x,y = self.simulate_move(angle_val[sensors_item],1)

                if self.count_grid[x][y] == 0:
                    sensor_weight = self.self_exploration_val 
                else:
                    if self.deadend_grid[x][y] == 1:
                        sensor_weight = self.epsilon_val
                    else:
                        sensor_weight = self.self_exploration_val **(-self.count_grid[x][y])

                weighted_array.append((sensors_item,sensor_weight))
            #For more space exploration, 
            rand_index = self.weighted_choice(weighted_array)

            rotation = angle_val[rand_index]
            movement = 1

            self.is_previous_loc_deadend = False
        else:
            #if that was the dead end case
            rotation = 90
            movement = 0
            self.update_deadends()
            self.is_previous_loc_deadend = True

        self.update_mapping(value) #Updates value for mapping

        #print self.isGoal(self.location)
        if self.is_all_space_explored():
            print 'All Space explored, ready for optimization'

            self.print_list(self.count_grid)
            print 'Total steps takes ',self.count_steps
            
            sys.exit()
        
        #Update robot location
        self.move(rotation, movement)
        
        return rotation, movement

    #Allowed actions for any mapped location of maze
    def allowed_actions(self,location):
        allowed_actions_list = []
        x,y = location

        #convert location to binary and then to list of integers
        allowed_actions_list = map(int,list('{0:04b}'.format(self.mapped_grid[x][y])))
        allowed_actions_list.reverse()

        #up right down left wall orientation

        possible_indexes = []
        for idx,action in enumerate(allowed_actions_list):
            if action == 1:
                possible_indexes.append(idx)

        return possible_indexes

    def build_heuristics(self):

        #For all of the currently opened maze positions
        stacked_positions = []

        for x in range(self.size_of_goal):
            for y in range(self.size_of_goal):
                location = (self.no_of_rows/2+x-1, self.no_of_cols/2+y-1)
                h_value = 0
                stacked_positions.append((location,h_value))
                self.update_heuristics(location, h_value)

        while(len(stacked_positions) >0):
            location , h_value = stacked_positions[0]
            del stacked_positions[0]

            allowed_actions_list = self.allowed_actions(location)

            for idx in allowed_actions_list:
                updated_loc = [ location[i]+forward[dir_index_ar[idx]][i] for i in range(self.no_of_dim)]
                if self.heuristics[x][y] != self.def_heu_val:
                    stacked_positions.append((updated_loc,h_value+1))
                    self.update_heuristics(updated_loc, h_value+1)


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
        self.count_steps +=1
        rotation, movement = self.robot_exploration(sensors)

        return rotation, movement
