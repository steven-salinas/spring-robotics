import copy
import time


import utils
import signal
import math
import RPi.GPIO as GPIO # README: https://pypi.org/project/RPi.GPIO/

import PID
import threading
global lSensor, fSensor, rSensor


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
grid_map = {1:'WWOW', 2:'OWOW', 3:'OWOO', 4:'OWWW',
            5:'WWOO', 6:'OWWO', 7:'WOWO', 8:'WWWO',
            9:'WOWO', 10:'WOOW', 11:'OOWW', 12:'WOWO',
            13:'WOOW', 14:'OWOW', 15:'OWOW', 16:'OOWW'}

# Motion Model
prob_forward = 0.8
prob_left = 0.05
prob_right = 0.05
prob_stay = 0.1

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
PARTICLE_PERCENT = 0.9
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
    global fSensor
    lSensor=utils.lSensor
    fSensor=utils.fSensor
    rSensor=utils.rSensor
    time.sleep(0.2)
    wall_threshold = 14
    mmToInch=0.0393701
    fDistance = fSensor.get_distance()*mmToInch
    lDistance = lSensor.get_distance()*mmToInch
    rDistance = rSensor.get_distance()*mmToInch
    #print(lDistance,fDistance,rDistance)
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
    print(readings)
    return readings

def wavefront(goal):
    arr = [[0,0,0,0],[0,0,0,0],[0,0,0,0],[0,0,0,0]]
    queue = []
    visited = []

    goal_idx = cell_to_index[goal]
    arr[goal_idx[0]][goal_idx[1]] = 2

    queue.append(goal)
    visited.append(goal)

    while queue:
        current_cell = queue.pop(0)
        current_idx = cell_to_index[current_cell]

        # Get info for current cell from map
        walls = grid_map[current_cell]

        # Check West cell
        if walls[0] == 'O':
            adjacent_cell = index_to_cell[current_idx[0],current_idx[1]-1]
            if adjacent_cell not in visited:
                visited.append(adjacent_cell)
                queue.append(adjacent_cell)
                arr[current_idx[0]][current_idx[1]-1] = arr[current_idx[0]][current_idx[1]] + 1

        # Check North cell
        if walls[1] == 'O':
            adjacent_cell = index_to_cell[current_idx[0]-1,current_idx[1]]
            if adjacent_cell not in visited:
                visited.append(adjacent_cell)
                queue.append(adjacent_cell)
                arr[current_idx[0]-1][current_idx[1]] = arr[current_idx[0]][current_idx[1]] + 1

        # Check East cell
        if walls[2] == 'O':
            adjacent_cell = index_to_cell[current_idx[0],current_idx[1]+1]
            if adjacent_cell not in visited:
                visited.append(adjacent_cell)
                queue.append(adjacent_cell)
                arr[current_idx[0]][current_idx[1]+1] = arr[current_idx[0]][current_idx[1]] + 1

        # Check South cell
        if walls[3] == 'O':
            adjacent_cell = index_to_cell[current_idx[0]+1,current_idx[1]]
            if adjacent_cell not in visited:
                visited.append(adjacent_cell)
                queue.append(adjacent_cell)
                arr[current_idx[0]+1][current_idx[1]] = arr[current_idx[0]][current_idx[1]] + 1

    '''for row in arr:
        print(row)'''
    return arr

def get_path(input_arr,beginning,goal):
    goal_idx = cell_to_index[goal]
    path = []
    visited = []
    beginning_arr=[beginning]
    while beginning != goal:
        current_idx = cell_to_index[beginning]
        direction = None
        min_cell = input_arr[current_idx[0]][current_idx[1]]

        # Get info for current cell from map
        walls = grid_map[beginning]

        # Check West cell
        if walls[0] == 'O':
            adjacent_cell = index_to_cell[current_idx[0],current_idx[1]-1]
            if input_arr[current_idx[0]][current_idx[1]-1] < min_cell and adjacent_cell not in visited:
                visited.append(adjacent_cell)
                direction = 0
                beginning = adjacent_cell
                min_cell = beginning

        # Check North cell
        if walls[1] == 'O':
            adjacent_cell = index_to_cell[current_idx[0]-1,current_idx[1]]
            if input_arr[current_idx[0]-1][current_idx[1]] < min_cell and adjacent_cell not in visited:
                visited.append(adjacent_cell)
                direction = 1
                beginning = adjacent_cell
                min_cell = beginning

        # Check East cell
        if walls[2] == 'O':
            adjacent_cell = index_to_cell[current_idx[0],current_idx[1]+1]
            if input_arr[current_idx[0]][current_idx[1]+1] < min_cell and adjacent_cell not in visited:
                visited.append(adjacent_cell)
                direction = 2
                beginning = adjacent_cell
                min_cell = beginning

        # Check South cell
        if walls[3] == 'O':
            adjacent_cell = index_to_cell[current_idx[0]+1,current_idx[1]]
            if input_arr[current_idx[0]+1][current_idx[1]] < min_cell and adjacent_cell not in visited:
                visited.append(adjacent_cell)
                direction = 3
                beginning = adjacent_cell
                min_cell = beginning

        path.append((beginning_arr[-1],direction))
        beginning_arr.append(beginning)
    return path

def get_closest_unvisited(visited,inputX,inputY):
    min_distance = 100000
    min_idx = (None,None)
    for row in range(ROWS):
        for col in range(COLUMNS):
            if index_to_cell[row,col] not in visited and inputX != col and inputY != row :
                distance = math.sqrt((inputX-row)**2 + (inputY-col)**2)
                if distance < min_distance:
                    min_distance = distance
                    min_idx = (row,col)
    return index_to_cell[min_idx[0],min_idx[1]]

def traverse(visited,cell,output_arr):
    time.sleep(0.2)
    global CURRENT_DIRECTION
    current_angle=utils.getIMUDegrees()
    rounded_angle=round(current_angle / 90) * 90

    walls = grid_map[cell]
    cell_idx = cell_to_index[cell]
    output_arr[cell_idx[0]][cell_idx[1]] = 'X'
    for row in output_arr:
        print(row)

    visited.add(cell)
    output_arr
    if len(visited) == 16:
        print("All cells visited")
        return

    print("Robot at cell {} heading {}".format(cell,directions[CURRENT_DIRECTION]))
    map_left = grid_map[cell][(CURRENT_DIRECTION-1)%4]
    map_front = grid_map[cell][CURRENT_DIRECTION]
    map_right = grid_map[cell][(CURRENT_DIRECTION+1)%4]
    #print(map_left,map_front,map_right)
    if map_front == 'O':
        utils.moveCell(18,2)
        if CURRENT_DIRECTION == 0:
            next_cell = index_to_cell[cell_idx[0],cell_idx[1]-1]
        if CURRENT_DIRECTION == 1:
            next_cell = index_to_cell[cell_idx[0]-1,cell_idx[1]]
        if CURRENT_DIRECTION == 2:
            next_cell = index_to_cell[cell_idx[0],cell_idx[1]+1]
        if CURRENT_DIRECTION == 3:
            next_cell = index_to_cell[cell_idx[0]+1,cell_idx[1]]
        if next_cell not in visited:
            traverse(visited,next_cell,output_arr)
            return
    if map_left == 'O':
        desired_angle=rounded_angle-90
        input_angle=desired_angle-current_angle
        utils.rotateA(input_angle)
        #utils.rotateA(-90)
        CURRENT_DIRECTION = (CURRENT_DIRECTION - 1) % 4
        utils.moveCell(18,2)
        if CURRENT_DIRECTION == 0:
            next_cell = index_to_cell[cell_idx[0],cell_idx[1]-1]
        if CURRENT_DIRECTION == 1:
            next_cell = index_to_cell[cell_idx[0]-1,cell_idx[1]]
        if CURRENT_DIRECTION == 2:
            next_cell = index_to_cell[cell_idx[0],cell_idx[1]+1]
        if CURRENT_DIRECTION == 3:
            next_cell = index_to_cell[cell_idx[0]+1,cell_idx[1]]
        if next_cell not in visited:
            traverse(visited,next_cell,output_arr)
            return

    if map_right == 'O':
        desired_angle=rounded_angle+90
        input_angle=desired_angle-current_angle
        utils.rotateA(input_angle)
        #utils.rotateA(90)
        CURRENT_DIRECTION = (CURRENT_DIRECTION + 1) % 4
        utils.moveCell(18,2)
        if CURRENT_DIRECTION == 0:
            next_cell = index_to_cell[cell_idx[0],cell_idx[1]-1]
        if CURRENT_DIRECTION == 1:
            next_cell = index_to_cell[cell_idx[0]-1,cell_idx[1]]
        if CURRENT_DIRECTION == 2:
            next_cell = index_to_cell[cell_idx[0],cell_idx[1]+1]
        if CURRENT_DIRECTION == 3:
            next_cell = index_to_cell[cell_idx[0]+1,cell_idx[1]]
        if next_cell not in visited:
            traverse(visited,next_cell,output_arr)
            return

    else:
        #closest_cell = get_closest_unvisited(visited, cell_idx[1], cell_idx[0])
        for i in range(1,17):
            if i not in visited:
                goal_cell = i
                break
        wavefront_arr = wavefront(goal_cell)
        path = get_path(wavefront_arr,cell,goal_cell)
        print(goal_cell,path)
        while path:
            time.sleep(0.2)
            current_move = path.pop(0)
            #print("Current move is {}".format(current_move))
            print("Current move is cell: {} heading {}".format(current_move[0],directions[current_move[1]]))
            #print(path)
            if CURRENT_DIRECTION != current_move[1]:
                angle = (CURRENT_DIRECTION - current_move[1])*-90
                print(angle)
                utils.rotateA(angle)
                CURRENT_DIRECTION = current_move[1]
            utils.moveCell(18,2)
            visited.add(current_move[0])
            current_idx = cell_to_index[current_move[0]]
            output_arr[current_idx[0]][current_idx[1]] = 'X'
            for row in output_arr:
                print(row)
            print("Robot at cell {} heading {}".format(current_move[0],directions[CURRENT_DIRECTION]))
        traverse(visited,goal_cell,output_arr)

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
    # readings = get_readings()
    # sensor_updated = sensor_update(readings,arr)
    # max_idx = get_max_particle(sensor_updated)
    # max_particle = sensor_updated[max_idx[0]][max_idx[1]]
    # print(max_particle, max_idx)
    motion_updated = arr

    while max_particle < PARTICLE_THRESHOLD:
        readings = get_readings()
        current_angle=utils.getIMUDegrees()
        rounded_angle=round(current_angle / 90) * 90
        # if readings[1] == 1:
        mmToInch=0.0393701
        fDistance = fSensor.get_distance()*mmToInch

        if fDistance < 18:
            sensor_updated = sensor_update(readings,motion_updated)
            max_idx = get_max_particle(sensor_updated)
            max_particle = sensor_updated[max_idx[0]][max_idx[1]]
            motion_updated = motion_update(sensor_updated)

            desired_angle=rounded_angle+90
            input_angle=desired_angle-current_angle
            utils.rotateA(input_angle)
            CURRENT_DIRECTION = (CURRENT_DIRECTION + 1) % 4
        else:

            sensor_updated = sensor_update(readings,motion_updated)
            max_idx = get_max_particle(sensor_updated)
            max_particle = sensor_updated[max_idx[0]][max_idx[1]]

            motion_updated = motion_update(sensor_updated)
            utils.moveCell(18,2)


    print("END LOCALIZATION HERE")

    visited = set()
    #visited.add((max_idx[0],max_idx[1]))
    #print("Visited is {}".format(visited))

    localized_cell = index_to_cell[max_idx]
    print("Robot is at: {} going {}".format(cell_to_coord[localized_cell], directions[CURRENT_DIRECTION]))
    #output_arr[max_idx[0]][max_idx[1]] = 'X'
    '''for row in output_arr:
        print(row)'''

    traverse(visited,localized_cell,output_arr)

    '''current_position = max_idx
    goal = get_closest_unvisited(visited,current_position[0],current_position[1])

    while len(visited) < ROWS*COLUMNS:
        wavefront_arr = wavefront(goal)
        print("Going from {} to {}".format(index_to_cell[current_position[0],current_position[1]],goal))
        path = get_path(wavefront_arr,index_to_cell[current_position[0],current_position[1]],goal)
        #print(path)
        while path:
            time.sleep(0.2)
            current_move = path.pop(0)
            print("Current move is {}".format(current_move))

            if CURRENT_DIRECTION != current_move[1]:
                angle = (CURRENT_DIRECTION - current_move[1])*-90

                utils.rotateA(angle)
                CURRENT_DIRECTION = current_move[1]

            utils.moveCell(18,2)
            output_arr[current_position[0]][current_position[1]] = 'X'
            visited.add(current_position)
            print("Visited is {}".format(visited))

            if(current_move[1] == 0):
                current_position = (current_position[0],current_position[1]-1)
            if(current_move[1] == 1):
                current_position = (current_position[0]-1,current_position[1])
            if(current_move[1] == 2):
                current_position = (current_position[0],current_position[1]+1)
            if(current_move[1] == 3):
                current_position = (current_position[0]+1,current_position[1])
        #print("Got here")
            goal = get_closest_unvisited(visited,current_position[0],current_position[1])
            print("Goal is:{}".format(goal))
        # print(goal)'''

    # + "going" '''+ directions[CURRENT_DIRECTION]'''
    '''print("Robot is at: {} going {}".format(cell_to_coord[localized_cell], directions[CURRENT_DIRECTION]))
    closest_cell = get_closest_unvisited(visited, output_arr,max_idx[0],max_idx[1])
    print("Finished localizing closest cell to {} is {}".format(localized_cell, closest_cell))
    wavefront_arr = wavefront(closest_cell)
    path = get_path(wavefront_arr,localized_cell,closest_cell)
    print("Path to goal is {}".format(path))'''

    #print(get_readings())
if __name__ == '__main__':
    main()
