import copy
import time

'''
import utils
import signal
import math
import RPi.GPIO as GPIO # README: https://pypi.org/project/RPi.GPIO/

import PID
import threading


def ctrlC(signum, frame):
    global breakFlag
    breakFlag=True
    print("Ctrl-C caught")

    # threadPID.join()
    print("Exiting")

    lSensor.stop_ranging()
    fSensor.stop_ranging()
    rSensor.stop_ranging()
    utils.setSpeedsPWM (1.508,1.5)
    GPIO.cleanup()
    exit()
'''

directions = ['W', 'N', 'E', 'S']
cell_to_index = {1:(0,0),2:(0,1),3:(0,2),4:(0,3),
                5:(1,0),6:(1,1),7:(1,2),8:(1,3),
                9:(2,0),10:(2,1),11:(2,2),12:(2,3),
                13:(3,0),14:(3,1),15:(3,2),16:(3,3)}
#print(cell_to_index[1])

index_to_cell = {(0,0):1,(0,1):2,(0,2):3,(0,3):4,
                (1,0):5,(1,1):6,(1,2):7,(1,3):8,
                (2,0):9,(2,1):10,(2,2):11,(2,3):12,
                (3,0):13,(3,1):14,(3,2):15,(3,3):16}
#print(index_to_cell[0,1])
surrounded_squares = {1:['W']}
adjacent_squares = {1:['N','S'],4:['N','E'],5:['N','W'],6:['N','E'],10:['W','S'],11:['E','S']
                ,13:['W','S'],16:['E','S']}
opposite_squares = {1:['E'],2:['E','W'],7:['N','S'],8:['N','S'],9:['N','S'],12:['N','S']
                ,14:['E','W'],15:['E','W']}
single_square = {2:['N','S'],3:['N'],4:['W'],5:['S'],6:['S'],7:['E','W'],8:['E','W']
                ,9:['E','W'],10:['E'],11:['W'],12:['E','W']
                ,13:['E'],14:['N','S'],15:['N','S'],16:['W']}

# Map of walls based on cell number and facing north
grid_map = {1:'WWOW', 2:'OWOW', 3:'OWOO', 4:'OWWO',
            5:'WWOO', 6:'OWWO', 7:'WOWO', 8:'WOWO',
            9:'WOWO', 10:'WOOW', 11:'OOWW', 12:'WOWO',
            13:'WOOW', 14:'OWOW', 15:'OWOW', 16:'OOWW'}

# Motion Model
prob_forward = 0.6
prob_left = 0.1
prob_right = 0.1
prob_stay = 0.2

# Sensor Model
prob_z0_s0 = 0.7
prob_z0_s1 = 0.1
prob_z1_s0 = 0.3
prob_z1_s1 = 0.9

# 0 = west, 1 = north, 2 = east, 3 = south
CURRENT_DIRECTION = 1

# Size of grid square in inches
SQ_SIZE = 18

# Number of beginning particles
PARTICLE_NUM = 112
PARTICLE_THRESHOLD = 0.9 * PARTICLE_NUM

ROWS = 4
COLUMNS = 4

def sensor_update(readings,input_arr:list[list[int]]) -> list[list[int]]:
    left_reading, front_reading, right_reading = readings[0],readings[1], readings[2]
    arr = [[0,0,0,0],[0,0,0,0],[0,0,0,0],[0,0,0,0]]
    arr_sum = 0
    #print(left_reading,front_reading,right_reading)
    for row in range(ROWS):
        for col in range(COLUMNS):
            cell = index_to_cell[row,col]
            left_prob, front_prob, right_prob = None, None, None
            map_left, map_front, map_right = None, None, None

            # Calculations for left of current cell
            if left_reading == 1:
                map_left = grid_map[cell][(CURRENT_DIRECTION-1)%4]
                if map_left == 'W':
                    left_prob = prob_z1_s1
                else:
                    left_prob = prob_z1_s0
            elif left_reading == 0:
                map_left = grid_map[cell][(CURRENT_DIRECTION-1)%4]
                if map_left == 'O':
                    left_prob = prob_z0_s0
                else:
                    left_prob = prob_z0_s1

            # Calculations for front of current cell  
            if front_reading == 1:
                map_front = grid_map[cell][CURRENT_DIRECTION]
                if map_front == 'W':
                    front_prob = prob_z1_s1
                else:
                    front_prob = prob_z1_s0
            elif front_reading == 0:
                map_front = grid_map[cell][CURRENT_DIRECTION]
                if map_front == 'O':
                    front_prob = prob_z0_s0
                else:
                    front_prob = prob_z0_s1

            # Calculations for right of current cell
            if right_reading == 1:
                map_right = grid_map[cell][(CURRENT_DIRECTION+1)%4]
                if map_right == 'W':
                    right_prob = prob_z1_s1
                else:
                    right_prob = prob_z1_s0
            elif right_reading == 0:
                map_right = grid_map[cell][(CURRENT_DIRECTION+1)%4]
                if map_right == 'O':
                    right_prob = prob_z0_s0
                else:
                    right_prob = prob_z0_s1
            
            # Probabilities from readings
            arr[row][col] = left_prob * front_prob * right_prob

            # Importance Factor
            arr[row][col] = arr[row][col] * input_arr[row][col]

            # Adding to sum for normalization
            arr_sum += arr[row][col]

    # Normalization
    for row in range(len(arr)):
        for col in range(len(arr[0])):
            arr[row][col] = arr[row][col] / arr_sum

    # Resampling
    for row in range(len(arr)):
        for col in range(len(arr[0])):
            arr[row][col] = round(arr[row][col] * PARTICLE_NUM)

    for row in arr:
        print(row)

    return arr

def motion_update(arr:list[list[int]], direction:str):
    for row in range(len(arr)):
        for col in range(len(arr[0])):
            # Going West
            if direction == 'W':
                if col - 1 >= 0:
                    arr[row][col-1] += arr[row][col-1] * prob_forward
                if row + 1 < ROWS:
                    arr[row+1][col] += arr[row+1][col] * prob_left
                if row - 1 >= 0:
                    arr[row-1][col] += arr[row-1][col] * prob_right
                arr[row][col] += arr[row][col] * prob_stay
            # Going North
            elif direction == 'N':
                if row - 1 >= 0:
                    arr[row-1][col] += arr[row-1][col] * prob_forward
                if col - 1 >= 0:
                    arr[row][col-1] += arr[row][col-1] * prob_left
                if col + 1 < COLUMNS:
                    arr[row][col + 1] += arr[row][col +1] * prob_right
                arr[row][col] += arr[row][col] * prob_stay
            # Going East
            elif direction == 'E':
                if col + 1 < COLUMNS:
                    arr[row][col+1] += arr[row][col+1] * prob_forward
                if row + 1 < ROWS:
                    arr[row+1][col] += arr[row+1][col] * prob_right
                if row - 1 >= 0:
                    arr[row-1][col] += arr[row-1][col] * prob_left
                arr[row][col] += arr[row][col] * prob_stay
            # Going South
            elif direction == 'S':
                if row + 1 < ROWS:
                    arr[row+1][col] += arr[row+1][col] * prob_forward
                if col - 1 >= 0:
                    arr[row][col-1] += arr[row][col-1] * prob_right
                if col + 1 < COLUMNS:
                    arr[row][col + 1] += arr[row][col +1] * prob_left
                arr[row][col] += arr[row][col] * prob_stay
    return arr


def main():
    # Cell number based on midpoint of cell coordinates
    coord_to_cell = {(-1.5*SQ_SIZE,1.5*SQ_SIZE):1, (-0.5*SQ_SIZE,1.5*SQ_SIZE):2, (0.5*SQ_SIZE,1.5*SQ_SIZE):3, (1.5*SQ_SIZE,1.5*SQ_SIZE):4,
                (-1.5*SQ_SIZE,0.5*SQ_SIZE):5, (-0.5*SQ_SIZE,0.5*SQ_SIZE):6, (0.5*SQ_SIZE,0.5*SQ_SIZE):7, (1.5*SQ_SIZE,0.5*SQ_SIZE):8,
                (-1.5*SQ_SIZE,-0.5*SQ_SIZE):9, (-0.5*SQ_SIZE,-0.5*SQ_SIZE):10, (0.5*SQ_SIZE,-0.5*SQ_SIZE):11, (1.5*SQ_SIZE,-0.5*SQ_SIZE):12,
                (-1.5*SQ_SIZE,-1.5*SQ_SIZE):13, (-0.5*SQ_SIZE,-1.5*SQ_SIZE):14, (0.5*SQ_SIZE,-1.5*SQ_SIZE):15, (1.5*SQ_SIZE,-1.5*SQ_SIZE):16}

    # Coordinates based on cell number
    cell_to_coord = {1:(-1.5*SQ_SIZE,1.5*SQ_SIZE), 2:(-0.5*SQ_SIZE,1.5*SQ_SIZE), 3:(0.5*SQ_SIZE,1.5*SQ_SIZE), 4:(1.5*SQ_SIZE,1.5*SQ_SIZE),
                    5:(-1.5*SQ_SIZE,0.5*SQ_SIZE), 6:(-0.5*SQ_SIZE,0.5*SQ_SIZE), 7:(0.5*SQ_SIZE,0.5*SQ_SIZE), 8:(1.5*SQ_SIZE,0.5*SQ_SIZE),
                    9:(-1.5*SQ_SIZE,-0.5*SQ_SIZE), 10:(-0.5*SQ_SIZE,-0.5*SQ_SIZE), 11:(0.5*SQ_SIZE,-0.5*SQ_SIZE), 12:(1.5*SQ_SIZE,-0.5*SQ_SIZE),
                    13:(-1.5*SQ_SIZE,-1.5*SQ_SIZE), 14:(-0.5*SQ_SIZE,-1.5*SQ_SIZE), 15:(0.5*SQ_SIZE,-1.5*SQ_SIZE), 16:(1.5*SQ_SIZE,-1.5*SQ_SIZE)}



    arr = [[0,0,0,0],[0,0,0,0],[0,0,0,0],[0,0,0,0]]

    SQ_NUM = ROWS * COLUMNS
    starting_particles = PARTICLE_NUM / SQ_NUM
    for i in range(len(arr)):
        for j in range(len(arr[0])):
            #print((i,j),cell_num.get((i,j)))
            #temp_cell_num = cell_num.get((i,j))
            #print(grid_map.get(temp_cell_num))
            arr[i][j]=starting_particles
    '''for row in arr:
        print(row)'''

    readings = [0,1,0]
    sensor_updated = sensor_update(readings,arr)
    #motion_updated_arr = motion_update(copy.deepcopy(arr),directions[CURRENT_DIRECTION])
    #print(motion_updated_arr[row])
    '''for column in range(len(arr[0])):
        #print(arr[row][column])
        pass'''

    '''belief_bar_arr = [[0]*COLUMNS]*ROWS
    for i in range(len(belief_bar_arr)):
        for j in range(len(belief_bar_arr)):
            belief_bar_arr[i][j] = arr[i][j] * motion_updated_arr[i][j]
        #print(belief_bar_arr[i])

    belief_arr = sensor_update(1,1,0,belief_bar_arr,starting_direction)'''
    '''for row in belief_arr:
        print(row)'''

    '''basic wall following using lab 3 task 4 code, then use encoders to measure each 18 inches, robot should stop when it
    it reaches a certain amount of ticks, use right wheel ticks as they are more constant'''
if __name__ == '__main__':
    main()
