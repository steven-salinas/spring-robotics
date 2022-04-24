import copy
import time


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

# Size of grid square in inches
SQ_SIZE = 18

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

# Number of beginning particles
PARTICLE_PERCENT = 0.8
PARTICLE_NUM = 112
PARTICLE_THRESHOLD = PARTICLE_PERCENT * PARTICLE_NUM

ROWS = 4
COLUMNS = 4

# 0 = west, 1 = north, 2 = east, 3 = south
CURRENT_DIRECTION = 1

def sensor_update(readings,input_arr):
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

    current_particles = 0
    global PARTICLE_NUM
    global PARTICLE_THRESHOLD
    # Resampling
    for row in range(len(arr)):
        for col in range(len(arr[0])):
            arr[row][col] = round(arr[row][col] * PARTICLE_NUM)
            current_particles += arr[row][col]
    
    # Updating particle number if changed
    if current_particles != PARTICLE_NUM:
        PARTICLE_NUM = current_particles
        PARTICLE_THRESHOLD = PARTICLE_PERCENT * PARTICLE_NUM

    print("\nAfter Sensor update and resampling")
    for row in arr:
        print(row)

    return arr

# Funcion to return the index with the max number of cells in array
def get_max_particle(input_arr):
    max_particle = -1
    max_idx = (-1,-1)
    for row in range(len(input_arr)):
        for col in range(len(input_arr[0])):
            if input_arr[row][col] > max_particle:
                max_particle = input_arr[row][col]
                max_idx = (row,col)
    return max_idx

# Function does motion update based on moving one cell forward
def motion_update(input_arr):
    global PARTICLE_NUM
    global PARTICLE_THRESHOLD
    arr = [[0,0,0,0],[0,0,0,0],[0,0,0,0],[0,0,0,0]]
    row = 0
    while row < 4:
        for col in range(len(input_arr[0])):
            #Skip if cell has no particles already
            if input_arr[row][col] == 0:
                continue

            cell = index_to_cell[row,col]
            prob_accumulated = prob_stay
            map_left = grid_map[cell][(CURRENT_DIRECTION-1)%4]
            map_front = grid_map[cell][CURRENT_DIRECTION]
            map_right = grid_map[cell][(CURRENT_DIRECTION+1)%4]
            
            if map_left == 'W':
                prob_accumulated += prob_left
            if map_front == 'W':
                prob_accumulated += prob_forward
            if map_right == 'W':
                prob_accumulated += prob_right
                

            # Adding particles for current cell
            arr[row][col] += round(input_arr[row][col] * prob_accumulated)

            ''' Will only work if outside of map is also walls
                As that the all check also works as an array bounds check'''

            # Going West
            if CURRENT_DIRECTION == 0:
                if map_front != 'W':
                    arr[row][col-1] += round(input_arr[row][col] * prob_forward)
                if map_left != 'W':
                    arr[row+1][col] += round(input_arr[row][col] * prob_left)
                if map_right != 'W':
                    arr[row-1][col] += round(input_arr[row][col] * prob_right)
            
            # Going North
            if CURRENT_DIRECTION == 1:
                if map_front != 'W':
                    arr[row-1][col] += round(input_arr[row][col] * prob_forward)
                if map_left != 'W':
                    arr[row][col-1] += round(input_arr[row][col] * prob_left)
                if map_right != 'W':
                    arr[row][col+1] += round(input_arr[row][col] * prob_right)

            # Going East
            if CURRENT_DIRECTION == 2:
                if map_front != 'W':
                    arr[row][col+1] += round(input_arr[row][col] * prob_forward)
                if map_left != 'W':
                    arr[row-1][col] += round(input_arr[row][col] * prob_left)
                if map_right != 'W':
                    arr[row+1][col] += round(input_arr[row][col] * prob_right)
            
            # Going South
            if CURRENT_DIRECTION == 3:
                if map_front != 'W':
                    arr[row+1][col] += round(input_arr[row][col] * prob_forward)
                if map_left != 'W':
                    arr[row][col+1] += round(input_arr[row][col] * prob_left)
                if map_right != 'W':
                    arr[row][col-1] += round(input_arr[row][col] * prob_right)
        row +=1
    
    # Outputting current matrix
    current_particle = 0
    print("\nAfter Motion update and resampling")
    for row in arr:
        print(row)
        current_particle += (sum(row))

    # Fix particle count if changed
    if current_particle != PARTICLE_NUM:
        PARTICLE_NUM = current_particle
        PARTICLE_THRESHOLD = PARTICLE_PERCENT * PARTICLE_NUM

    return arr

def get_readings():
    lSensor=utils.lSensor
    fSensor=utils.fSensor
    rSensor=utils.rSensor
    time.sleep(0.2)
    wall_threshold = 9
    mmToInch=0.0393701
    fDistance = fSensor.get_distance()*mmToInch
    lDistance = lSensor.get_distance()*mmToInch
    rDistance = rSensor.get_distance()*mmToInch
    print(lDistance,fDistance,rDistance)
    readings = [None,None,None]
    if lDistance < wall_threshold:
        readings[0] = 1
    else:
        readings[0] = 0 

    if fDistance < wall_threshold:
        readings[1] = 1
    else:
        readings[1] = 0

    if rDistance < wall_threshold:
        readings[2] = 1
    else:
        readings[2] = 0
    return readings

def main():
    global CURRENT_DIRECTION
    global PARTICLE_THRESHOLD
    global breakFlag
    global pidL, pidR
    global lSensor, fSensor, rSensor
    global startSpeedL, startSpeedR

    breakFlag=False
    signal.signal(signal.SIGINT, ctrlC)
    utils.initEncoders()
    utils.initMotors()
    utils.initPID(0,0,0)
    utils.initTOF()
    

    arr = [[0,0,0,0],[0,0,0,0],[0,0,0,0],[0,0,0,0]]
    output_arr = [['0','0','0','0'],['0','0','0','0'],['0','0','0','0'],['0','0','0','0']]
    max_particle = -1

    SQ_NUM = ROWS * COLUMNS
    starting_particles = PARTICLE_NUM / SQ_NUM
    for i in range(len(arr)):
        for j in range(len(arr[0])):
            arr[i][j]=starting_particles
    
    # Cell 12 or with real robot the first reading
    readings = get_readings()
    sensor_updated = sensor_update(readings,arr)
    max_idx = get_max_particle(sensor_updated)
    max_particle = sensor_updated[max_idx[0]][max_idx[1]]
    print(max_particle, max_idx)
    motion_updated = motion_update(sensor_updated)

    while max_particle < PARTICLE_THRESHOLD:        
        readings = get_readings()
        if readings[1] == 1:
            sensor_updated = sensor_update(readings,motion_updated)
            max_idx = get_max_particle(sensor_updated)
            max_particle = sensor_updated[max_idx[0]][max_idx[1]]

            motion_updated = motion_update(sensor_updated)
            utils.rotateA(90)
            CURRENT_DIRECTION = (CURRENT_DIRECTION + 1) % 4
        else:
            utils.moveXV(18,2)
            sensor_updated = sensor_update(readings,motion_updated)
            max_idx = get_max_particle(sensor_updated)
            max_particle = sensor_updated[max_idx[0]][max_idx[1]]

            motion_updated = motion_update(sensor_updated)
        '''Simulating input from going from cell 12 up to cell 4,
        notice how cell 4 has two readings, as max particle reading is taken
        before turning 90 degrees clockwise'''
        
        '''# Cell 12
        readings = [1,0,1]
        sensor_updated = sensor_update(readings,arr)
        max_idx = get_max_particle(sensor_updated)
        max_particle = sensor_updated[max_idx[0]][max_idx[1]]
        motion_updated = motion_update(sensor_updated)
        print(max_particle, max_idx)

        # 8
        sensor_updated = sensor_update(readings,motion_updated)
        motion_updated = motion_update(sensor_updated)
        max_idx = get_max_particle(sensor_updated)
        max_particle = sensor_updated[max_idx[0]][max_idx[1]]
        print(max_particle, max_idx)

        # 4 taken before turning 90 clockwise
        readings = [0,1,1]
        sensor_updated = sensor_update(readings,motion_updated)
        max_idx = get_max_particle(sensor_updated)
        max_particle = sensor_updated[max_idx[0]][max_idx[1]]
        motion_updated = motion_update(sensor_updated)
        # Turn(90)
        print(max_particle, max_idx)
        CURRENT_DIRECTION = CURRENT_DIRECTION = (CURRENT_DIRECTION + 1) % 4

        # 4 taken before turning 90 clockwise
        readings = [1,1,0]
        sensor_updated = sensor_update(readings,motion_updated)
        max_idx = get_max_particle(sensor_updated)
        max_particle = sensor_updated[max_idx[0]][max_idx[1]]
        motion_updated = motion_update(sensor_updated)
        
        print(max_particle, max_idx)'''

        print("END LOCALIZATION HERE")

    localized_cell = index_to_cell[max_idx]
    # + "going" '''+ directions[CURRENT_DIRECTION]'''
    print("Robot is at: {} going {}".format(cell_to_coord[localized_cell], directions[CURRENT_DIRECTION]))

    '''TODO
    - Naviagate around the rest of the squares once localized'''
    output_arr[max_idx[0]][max_idx[1]] = 'X'
    for row in output_arr:
        print(row)
    print(get_readings())
if __name__ == '__main__':
    main()
