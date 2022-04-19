import copy
import utils
import time

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
surrounded_squares = {1:['W']}
adjacent_squares = {1:['N','S'],4:['N','E'],5:['N','W'],6:['N','E'],10:['W','S'],11:['E','S']
                ,13:['W','S'],16:['E','S']}
opposite_squares = {1:['E'],2:['E','W'],7:['N','S'],8:['N','S'],9:['N','S'],12:['N','S']
                ,14:['E','W'],15:['E','W']}
single_square = {2:['N','S'],3:['N'],4:['W'],5:['S'],6:['S'],7:['E','W'],8:['E','W']
                ,9:['E','W'],10:['E'],11:['W'],12:['E','W']
                ,13:['E'],14:['N','S'],15:['N','S'],16:['W']}

# Motion Model
prob_forward = 0.7
prob_left = 0.1
prob_right = 0.1
prob_stay = 0.2

# Size of grid square in inches
SQ_SIZE = 18

ROWS = 4
COLUMNS = 4

def sensor_update(left:int,front:int,right:int,input_arr:list[list[int]],dir:str) -> list[list[int]]:
    reading = [left,front,right]
    selection = None
    if reading == [1,1,1]:
        selection = surrounded_squares
    elif reading == [1,1,0] or reading == [0,1,1]:
        selection = adjacent_squares
    elif reading == [1,0,1]:
        selection = opposite_squares
    elif reading == [1,0,0] or reading == [0,1,0] or reading == [0,0,1]:
        selection = single_square
    else:
        selection = None

    belief_arr = [[0,0,0,0],[0,0,0,0],[0,0,0,0],[0,0,0,0]]
    cell_prob = 1/len(selection.keys())
    for key in selection.keys():        
        i,j = cell_to_index[key]
        if dir in selection[key]:
            belief_arr[i][j] = cell_prob * input_arr[i][j]

    arr_sum = 0
    for row in belief_arr:
        arr_sum += sum(row)
    
    # Normalize
    for row in range(len(belief_arr)):
        for column in range(len(belief_arr[0])):
            if belief_arr[row][column] != 0:
                belief_arr[row][column] = belief_arr[row][column] / arr_sum
    
    for row in belief_arr:
        print(row)
    return belief_arr

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

    # Map of walls based on cell number and facing north
    grid_map = {1:'WWOW', 2:'OWOW', 3:'OWOO', 4:'OWWO',
                5:'WWOO', 6:'OWWO', 7:'WOWO', 8:'WOWO',
                9:'WOWO', 10:'WOOW', 11:'OOWW', 12:'WOWO',
                13:'WOOW', 14:'OWOW', 15:'OWOW', 16:'OOWW'}

    arr = [[0,0,0,0],[0,0,0,0],[0,0,0,0],[0,0,0,0]]

    NUM_SQ = ROWS * COLUMNS
    starting_probability = 1/NUM_SQ
    starting_direction = directions[1] # North
    #print(starting_possibility)
    #print(cell_num.keys())
    for i in range(len(arr)):
        for j in range(len(arr[0])):
            #print((i,j),cell_num.get((i,j)))
            #temp_cell_num = cell_num.get((i,j))
            #print(grid_map.get(temp_cell_num))
            arr[i][j]=starting_probability

    motion_updated_arr = motion_update(copy.deepcopy(arr),starting_direction)

    for row in range(len(motion_updated_arr)):
        #print(arr[row])
        #print(motion_updated_arr[row])
        '''for column in range(len(arr[0])):
            #print(arr[row][column])
            pass'''

    belief_bar_arr = [[0]*COLUMNS]*ROWS
    for i in range(len(belief_bar_arr)):
        for j in range(len(belief_bar_arr)):
            belief_bar_arr[i][j] = arr[i][j] * motion_updated_arr[i][j]
        #print(belief_bar_arr[i])

    belief_arr = sensor_update(1,1,0,belief_bar_arr,starting_direction)
    '''for row in belief_arr:
        print(row)'''
    
    '''basic wall following using lab 3 task 4 code, then use encoders to measure each 18 inches, robot should stop when it
    it reaches a certain amount of ticks, use right wheel ticks as they are more constant'''
if __name__ == '__main__':
    main()