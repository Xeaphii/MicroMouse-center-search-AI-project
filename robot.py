import numpy as np
import time
import sys
from random import random
from bisect import bisect

forward = [[-1,  0], # go up
           [ 0, -1], # go left
           [ 1,  0], # go down
           [ 0,  1]] # go right
forward_name = ['up', 'right', 'down', 'left']
#forward_name = ['^', '>', 'v', '<']

# action has 3 values: right turn, no turn, left turn
action = [1, 0, -1]
action_name = ['L', '#', 'R']

dir_names   = ['up', 'left', 'down', 'right']
angle_val = [-90, 0,90]
rev_dir_index= [1,2,1,0]

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
         
        self.init_heuristics = [[self.def_heu_val for row in range(self.no_of_rows)]
                 for col in range(self.no_of_cols)]
                 
        self.route = []

        x,y = self.location
        #self.count_grid[x][y] = 1
        self.is_previous_loc_deadend = False
        self.self_exploration_val    = 10.
        self.epsilon_val = 0.0000001
        self.count_steps = 0
        self.size_of_goal= 2
        self.no_of_dim   = 2
        self.rotate_cost = 1
        self.normal_cost = 1
        self.is_exploration_done = False
        self.is_changed_explorat = False
        self.steps_count = 0   # used for optimal path
        self.deadends_weight = 10000
        
        #Making initial heuristics for exploration
        self.initial_heuristics()

    #For checking percentage coverage
    def test_coverage():
        sum_of_values = 0
        for i in range(len(self.count_grid)):
            for j in range(len(self.count_grid[0])):
                if self.count_grid[i][j] !=0:
                    sum_of_values +=1
        normalized_coverage = sum_of_values/(len(self.count_grid) * len(self.count_grid[0]))
        return normalized_coverage

    #Return whether goal is acheived 
    def isGoal(self, location):
        row_no, col_no = location
        return (self.no_of_rows/2-1) <= row_no \
                and row_no <= (self.no_of_rows/2) and \
               (self.no_of_cols/2-1) <= col_no \
               and col_no <= (self.no_of_cols/2)

    # for checking whether whole maze is explored while in the exploration phase
    def is_all_space_explored(self):
        for i in range(len(self.count_grid)):
            for j in range(len(self.count_grid[0])):
                if self.count_grid[i][j] ==0:
                    return False
        return True

    #Adjusted move for robot for keeping track of internal state and respective variables
    def move(self, steering, distance):

        # make a new copy
        curr_pos  = dir_names.index(self.heading)
        act_index = angle_val.index(steering)
        dir_index = (curr_pos+action[act_index])%len(dir_names)
        self.heading = dir_names[dir_index]

        delta     = forward[dir_index] 
        self.location  = [ self.location[i]+delta[i]*distance for i in range(2) ]
        #return    (direction, location)

    #Helper method for printing the list of list i.e matriz 
    def print_list(self,input_list):
        for inner_list in input_list:
            print inner_list,'\n'
        print '\n \n'

    #helper method for updating counter grid
    def update_counter(self):
        #Updating location count     
        x,y = self.location
        self.count_grid[x][y]  += 1

    #helper method for updating dead ends grid
    def update_deadends(self):
        #Updating location count     
        x,y = self.location
        self.deadend_grid[x][y]  += 1

    #helper method for updating heuristics grid
    def update_heuristics(self,location,h_value):
        #Updating location count     
        x,y = location
        self.heuristics[x][y] = h_value

    #helper method for updating mapping grid
    def update_mapping(self,value):
        #Updating location count     
        # - up = 1 = 2^0 = 2^Direction.N.value 
        # - right  = 2 = 2^1 = 2^Direction.E.value
        # - down = 3 = 2^2 = 2^Direction.S.value
        # - left  = 4 = 2^3 = 2^Direction.W.value

        x,y = self.location
        self.mapped_grid[x][y] = value

    #Helper method for doing weighted average of choices
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

    #Simulate move function without updating robot internal states and respective variables
    def simulate_move(self, steering, distance):

        # make a new copy
        curr_pos  = dir_names.index(self.heading)
        act_index = angle_val.index(steering)
        dir_index = (curr_pos+action[act_index])%len(dir_names)

        delta     = forward[dir_index] 
        return [ self.location[i]+delta[i]*distance for i in range(2) ]

    #Helper method for getting corrected robot orientation
    def get_corrected_orientation(self, steering):

        # make a new copy
        curr_pos  = dir_names.index(self.heading)
        act_index = steering
        dir_index = (curr_pos+action[act_index])%len(dir_names)
        return dir_index_ar[dir_index]

        
    # Weighted explorator for count probability
    def weighted_prob_exploration(self,sensors_array):
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
        return self.weighted_choice(weighted_array)

    # Weighted explorator for count probability
    def weighted_prob_exploration_heu(self,sensors_array):
        weighted_array = []
        for sensors_item in sensors_array:
            x,y = self.simulate_move(angle_val[sensors_item],1)

            if self.count_grid[x][y] == 0:
                sensor_weight = self.self_exploration_val  + self.init_heuristics[x][y]
            else:
                if self.deadend_grid[x][y] == 1:
                    sensor_weight = self.epsilon_val 
                else:
                    sensor_weight = (self.self_exploration_val+ self.init_heuristics[x][y]) **(-self.count_grid[x][y])
                    

            weighted_array.append((sensors_item,sensor_weight))
        #For more space exploration, 
        return self.weighted_choice(weighted_array)
        
    # Weighted explorator for count probability
    def counting_exploration(self,sensors_array):
        weighted_array = []
        sensor_mapped  = []
        for sensors_item in sensors_array:
            x,y = self.simulate_move(angle_val[sensors_item],1)

            if self.deadend_grid[x][y] == 0:
                weighted_array.append(self.count_grid[x][y])
                sensor_mapped.append(sensors_item)
            else:
                #Needed to be removed later
                #print 'Reached in the dead inside array of length 0, sensors ',sensors_array,' at loc ',( x,y),' count '
                #sys.exit()
                #Needed to be removed later
                #return 1
                weighted_array.append(self.deadends_weight) #Avoid next deadends
                sensor_mapped.append(sensors_item)
                
        min_count = sensor_mapped[weighted_array.index(min(weighted_array))]

        return min_count
        
        
    # Weighted explorator for count probability
    def counting_exploration_heuristic(self,sensors_array):
        weighted_array = []
        sensor_mapped  = []
        for sensors_item in sensors_array:
            x,y = self.simulate_move(angle_val[sensors_item],1)

            if self.deadend_grid[x][y] == 0:
                weighted_array.append(self.count_grid[x][y] + self.init_heuristics[x][y])
                sensor_mapped.append(sensors_item)
            else:
                #Needed to be removed later
                #print 'Reached in the dead inside array of length 0, sensors ',sensors_array,' at loc ',( x,y),' count '
                #sys.exit()
                #Needed to be removed later
                #return 1
                weighted_array.append(self.deadends_weight) #Avoid next deadends
                sensor_mapped.append(sensors_item)
                
        min_count = sensor_mapped[weighted_array.index(min(weighted_array))]

        return min_count
    
    #Helper function for checking whether all space is explored or not
    def all_space_explorere(self):
        return self.is_all_space_explored() or self.count_steps >=900
        
    #Helper function for checking whether all space is explored or not
    def goal_first_explorere(self):
        return self.isGoal(self.location) or self.count_steps >=900

    #for robot exploration in the first run
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

            # Place for brain of exploration
            #rand_index = self.weighted_prob_exploration(sensors_array)

            #For counting explorator without wieghted probability
            #rand_index = self.counting_exploration(sensors_array)
            
            #For counting explorator without wieghted probability
            rand_index = self.counting_exploration_heuristic(sensors_array)
            
            #For counting explorator with wieghted probability and heuristics
            #rand_index = self.weighted_prob_exploration_heu(sensors_array)            

            #print 'rand_index ',rand_index
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

        #print for all space exploration
        #if self.all_space_explorere():
        
        # for goal first explorere
        if self.goal_first_explorere():
        
            print 'All Space explored, ready for optimization steps are ',self.count_steps
            
            #setting check for moving onto the second run
            self.is_exploration_done = True
            
            #self.print_list(self.count_grid)
            #print 'Total steps takes ',self.count_steps

            #Now moving for second run
            #time.sleep(0.5)
            
            #setting is changed exploration for running route search method 
            self.is_changed_explorat = True   

            #print 'Now printing heuristics'
            #self.print_list(self.heuristics)

            #print 'Now performing a star search'
            #action_grid,actions_list = self.a_star_search()
            #self.print_list(action_grid)
            
            #Defining path for robot
            #route,steps = self.get_route(actions_list)
            
            #self.print_list(route)
            #print 'Done in steps: ',steps
            
            #sys.exit()
            rotation = 'Reset'
            movement = 'Reset'
        #Update robot location
        if self.is_exploration_done != True:
            self.move(rotation, movement)
        
        return rotation, movement
    
    #Method to get route from given action list
    def get_route(self,action_list):
        
        prev_action = 0 # As the robot is faced up at start location
        move_list   = []
        #Doing some initialization here
        self.heading     = 'up'
        self.location    = self.init_loc
        idx = 0
        count = 0
        while idx < len(action_list):
            count +=1
            index_loc = dir_names.index(action_list[idx])
            index_loc = index_loc - prev_action
            if index_loc == 3 or index_loc == -1:
                rotation = 90
            elif index_loc == -3 or index_loc == 1:
                rotation = -90
            else:
                rotation = 0
            #rotation  = index_loc * 90
            
            #Updating robot location
            movement = 1
            #self.move(rotation, movement)
            count_changed = 0
            for i in range(1,3):
                if (idx+i < len(action_list) and action_list[idx] == action_list[idx+i] and count_changed == 0) or (count_changed == 1 and movement > 1):
                    movement +=1
                    
                elif idx+i < len(action_list) and action_list[idx] != action_list[idx+i]:
                    count_changed += 1
                    
            idx += (movement - 1) 
            move_list.append([rotation,movement])
            
            prev_action = dir_names.index(action_list[idx])
            idx +=1
        return move_list,count    

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


    #For building heuristics function from mapped maze
    def build_heuristics(self):
        
        #Doing some initialization here
        prev_action = -1
        
        #For all of the currently opened maze positions
        stacked_positions = []

        for x in range(self.size_of_goal):
            for y in range(self.size_of_goal):
                location = (self.no_of_rows/2+x-1, self.no_of_cols/2+y-1)
                h_value = 0
                stacked_positions.append((location,h_value,prev_action))
                self.update_heuristics(location, h_value)

        while(len(stacked_positions) >0):
            
                         #Set it to some value later
            #if prev_action == -1:
                
            location , h_value,prev_action = stacked_positions.pop(0)

           
            allowed_actions_list = self.allowed_actions(location)
                
            #Needed to remove later
            #print 'location ',location
            #if location == [0,4]:
            #    print 'Reached unknown loc, allowed Loc ',allowed_actions_list
            #    sys.exit()
            #Needed to remove later
             
            for idx in allowed_actions_list:
                
                #Keeping trakc of robot new orientation
                
                #index_loc = dir_names.index(action_list[idx])
                
                updated_loc = [ location[i]+forward[dir_index_ar[idx]][i] for i in range(self.no_of_dim)]
                
                if self.heuristics[updated_loc[0]][updated_loc[1]] == self.def_heu_val and self.mapped_grid[updated_loc[0]][updated_loc[1]] !=0:
                
                    movement = 1
                    rotation = 0
                    
                    self.move(rotation, movement) 
                    local_cost = 1 if prev_action == idx or prev_action == -1 else 2
                    stacked_positions.append((updated_loc,h_value+local_cost,idx))
                    self.update_heuristics(updated_loc, h_value+local_cost)


    #Cost method for returing cost of turn 
    def cost(self,index):
        #If the location is in the forward or backward direction
        if index == 0 or index == 2:
            return self.normal_cost
        else:
            return self.rotate_cost


    #Searching using A star method for optimal path from start to end
    def a_star_search(self):

        closed = [[0 for col in range(self.no_of_cols)] for row in range(self.no_of_rows)]
        closed[self.init_loc[0]][self.init_loc[1]] = 1
        
        #Action list for actions in sequence
        actions_list = []
        
        
        action = [[' ' for col in range(self.no_of_cols)] for row in range(self.no_of_rows)]

        x,y = self.init_loc
        g = 0
        h = self.heuristics[x][y]
        open = [[g+h,g, x, y]]
        
        #Doing some initialization here
        #self.heading     = 'up'
        #self.location    = self.init_loc

        found = False  # flag that is set when search is complete
        resign = False
        count = 0
        
        while not found and not resign:
        
            #Needed to be removed later
            print 'Count in the A star ',count,' open ',open,' '
            #time.sleep(2)
            #Needed to be removed later
            
            #time.sleep(5)
            if len(open) == 0:
                resign = True
                return "Fail"
            else:
                open.sort()
                open.reverse()
                next = open.pop()
                x = next[2]
                y = next[3]
                location = (x,y)
                g2  = next[0]
                
                
                if self.isGoal((x,y)):
                    found = True
                else:
                    count += 1 #Its location should be here
                    allowed_actions_list = self.allowed_actions((x,y))
                    min_action_value = float('Inf')
                    min_action_index = 0
                    
                    #Needed to remove later
                    print 'allowed_actions_list ',allowed_actions_list,' location ',(x,y),' mapped ',self.mapped_grid[x][y]
                    #Needed to remove later
                    
                    for idx in allowed_actions_list:
                        g = 0
                        #print 'allowed_actions_list ',allowed_actions_list
                        updated_loc = [ location[i]+forward[dir_index_ar[idx]][i] for i in range(self.no_of_dim)]
                        
                        
                        #Needed to remove later  
                        print 'possible states ',updated_loc,' g value ',g,' closed ',closed[updated_loc[0]][updated_loc[1]],' allowed ',allowed_actions_list
                        #Needed to remove later   
                        
                        if closed[updated_loc[0]][updated_loc[1]] == 0 and self.mapped_grid[updated_loc[0]][updated_loc[1]] != 0:
                            g = self.cost(idx) + self.heuristics[updated_loc[0]][updated_loc[1]]
                            
                            #Needed to remove later  
                            #print 'possible states ',updated_loc,' g value ',g
                            #Needed to remove later    
                            
                            
                            if  g< min_action_value:
                                min_action_value = g
                                min_x2 ,min_y2   = updated_loc
                                min_action_index = idx
                            
                            
                    #Needed to remove later       
                    print 'Chosen location ',(min_x2 ,min_y2)
                    if min_action_value == float('Inf'):
                        #self.print_list(action)
                        self.print_list(self.heuristics)
                        self.print_list(self.mapped_grid)
                        print 'Donot find any location'
                        sys.exit()
                    #Needed to remove later  
                                
                    action[x][y] = forward_name[min_action_index]#count#forward_arrows[dir_index_ar[idx]] 
                    #actions_list.append(forward_name[min_action_index])
                    actions_list.append(forward_name[min_action_index])
                    

                    open.append([g2+min_action_value,g2, min_x2, min_y2])
                    closed[min_x2][min_y2] = 1
                    print '\n'
                    
        print 'Total steps without multiple movements taken are ',count
        return action,actions_list
       
    #For getting planned optimal path obtained from mapped maze
    def planned_movement(self,sensors):
        rotation = 0
        movement = 1
        if self.is_changed_explorat:
        
            #Setting values for initial robot state
            self.heading     = 'up'
            self.location    = self.init_loc
            
            #building heuristics
            self.build_heuristics()
            
            #Needed to be removed later
            self.print_list(self.heuristics)
            #Needed to be removed later
            
            #print 'Now performing a star search'
            action_grid,actions_list = self.a_star_search()
            #Needed to be removed later
            self.print_list(action_grid)
            #Needed to be removed later
            
            #self.print_list(action_grid)
            #Defining path for robot
            route,steps = self.get_route(actions_list)
            self.route = route
            
            #setting is changed exploration function to false for not running these method again
            self.is_changed_explorat = False  
            
            print 'Total steps with multiple steps ', len(self.route)
        
        if len(self.route)>1 and len(self.route) > self.steps_count:
            rotation, movement = self.route[self.steps_count]
        
        if self.isGoal(self.location):
            print 'Reached goal'
        self.move(rotation, movement)    
            
        self.steps_count +=1
        
        return int(rotation), int( movement)
        
    #Method for making initial heuristics values without mapping
    def initial_heuristics(self):
        for x in range(0,self.no_of_cols/2):
            for i in  range(x,self.no_of_cols-x):
                for j in range(x,self.no_of_rows-x):
                    self.init_heuristics[i][j] = ((self.no_of_cols/2) - x -1)**0.5

    #Main method for moving robot
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
        if self.is_exploration_done:
            rotation, movement = self.planned_movement (sensors)
        else:
            rotation, movement = self.robot_exploration(sensors)
            
        #Needed to be removed later
        #print rotation, movement
        
        print 'on step ',self.count_steps
        #Needed to be removed later
        return rotation, movement
        