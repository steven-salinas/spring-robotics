import copy
import time
import signal
import math
import threading
import matplotlib.pyplot as plt

import PID
import RPi.GPIO as GPIO # README: https://pypi.org/project/RPi.GPIO/
import utils
import collections

def ctrlC(signum, frame):
    global breakFlag
    utils.setSpeedsPWM (1.508,1.5)
    breakFlag=True
    print("Ctrl-C caught")

    # threadPID.join()
    print("Exiting")

    lSensor.stop_ranging()
    fSensor.stop_ranging()
    rSensor.stop_ranging()

    GPIO.cleanup()
    exit()



ROWS = 4
COLUMNS = 4

# 0 = west, 1 = north, 2 = east, 3 = south
CURRENT_DIRECTION = 1



def get_readings():
    global fSensor,lSensor,rSensor

    time.sleep(0.2)
    wall_threshold = 14
    mmToInch=0.0393701
    fDistance = fSensor.get_distance()*mmToInch
    lDistance = lSensor.get_distance()*mmToInch
    rDistance = rSensor.get_distance()*mmToInch
    # print(lDistance,fDistance,rDistance)
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
    # print(readings)
    return readings

def visulize_ordered_map(ordered_map):
    neighbor_cells_delta=((0,-1),(-1,0),(0,1),(1,0))

    visualize_map=[
    [1,1,1,1,1,1,1,1,1],
    [1,1,1,1,1,1,1,1,1],
    [1,1,1,1,1,1,1,1,1],
    [1,1,1,1,1,1,1,1,1],
    [1,1,1,1,1,1,1,1,1],
    [1,1,1,1,1,1,1,1,1],
    [1,1,1,1,1,1,1,1,1],
    [1,1,1,1,1,1,1,1,1],
    [1,1,1,1,1,1,1,1,1],
    ]

    cell_idx_arr=[
    (7,1),(5,1),(3,1),(1,1),
    (7,3),(5,3),(3,3),(1,3),
    (7,5),(5,5),(3,5),(1,5),
    (7,7),(5,7),(3,7),(1,7),
    ]
    for i in range(len(ordered_map)):
        #
        # print(ordered_map[i][0],cell_idx_arr[i])
        current_cell=cell_idx_arr[i]
        # print(visualize_map[current_cell[0]][current_cell[1]])
        visualize_map[current_cell[0]][current_cell[1]]=0
        for delta_idx in range(len(neighbor_cells_delta)):
            neighbor_cell=(current_cell[0]+neighbor_cells_delta[delta_idx][0],current_cell[1]+neighbor_cells_delta[delta_idx][1])
            print(current_cell,ordered_map[i][0],neighbor_cell,ordered_map[i][1][delta_idx],ordered_map[i][1])

            visualize_map[neighbor_cell[0]][neighbor_cell[1]]=ordered_map[i][1][delta_idx]




    for i in range(len(visualize_map)):
        for j in range(len(visualize_map[0])):
            if visualize_map[i][j]==None:
                visualize_map[i][j]=-1

    plt.imshow(visualize_map)
    plt.draw()
    plt.pause(0.001)


signal.signal(signal.SIGINT, ctrlC)
def main():
    global CURRENT_DIRECTION
    global PARTICLE_THRESHOLD
    global breakFlag
    global pidL, pidR
    global lSensor, fSensor, rSensor
    global startSpeedL, startSpeedR

    breakFlag=False

    utils.initEncoders()
    utils.initMotors()
    utils.initPID(0,0,0)
    utils.initTOF()

    lSensor=utils.lSensor
    fSensor=utils.fSensor
    rSensor=utils.rSensor

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
    # grid_map = {1:'WWOW', 2:'OWOW', 3:'OWOO', 4:'OWWO',
    #             5:'WWOO', 6:'OWWO', 7:'WOWO', 8:'WOWO',
    #             9:'WOWO', 10:'WOOW', 11:'OOWW', 12:'WOWO',
    #             13:'WOOW', 14:'OWOW', 15:'OWOW', 16:'OOWW'}


    arr = [[0,0,0,0],[0,0,0,0],[0,0,0,0],[0,0,0,0]]
    output_arr = [['0','0','0','0'],['0','0','0','0'],['0','0','0','0'],['0','0','0','0']]
    max_particle = -1

    SQ_NUM = ROWS * COLUMNS



    # Cell 12 or with real robot the first reading
    # readings = get_readings()
    # sensor_updated = sensor_update(readings,arr)
    # max_idx = get_max_particle(sensor_updated)
    # max_particle = sensor_updated[max_idx[0]][max_idx[1]]
    # print(max_particle, max_idx)

    # grid_map={ (0,-1):(1,1,1,1)}
    grid_map={}
    motion_updated = arr

    start_position=[0,0]

    CURRENT_POSITION=start_position

    neighbor_cells_delta=((-1,0),(0,1),(1,0),(0,-1))

    visualize_map=[
    [1,1,1,1,1,1,1,1,1],
    [1,1,1,1,1,1,1,1,1],
    [1,1,1,1,1,1,1,1,1],
    [1,1,1,1,1,1,1,1,1],
    [1,1,1,1,1,1,1,1,1],
    [1,1,1,1,1,1,1,1,1],
    [1,1,1,1,1,1,1,1,1],
    [1,1,1,1,1,1,1,1,1],
    [1,1,1,1,1,1,1,1,1],
    ]

    cell_idx_arr=[
    (7,1),(5,1),(3,1),(1,1),
    (7,3),(5,3),(3,3),(1,3),
    (7,5),(5,5),(3,5),(1,5),
    (7,7),(5,7),(3,7),(1,7),
    ]

    while len(grid_map)<16:
        readings = get_readings()
        # readings=[1,0,1]

        map_update_value=[None,None,None,None]

        if (CURRENT_POSITION[0],CURRENT_POSITION[1]) in grid_map:
            map_update_value=grid_map[CURRENT_POSITION[0],CURRENT_POSITION[1]]
        #Checks if neighbor cells exist and what their wall values are, if they exist preload them
        for delta_idx in range(len(neighbor_cells_delta)):
            neighbor_cell=(CURRENT_POSITION[0]+neighbor_cells_delta[delta_idx][0],CURRENT_POSITION[1]+neighbor_cells_delta[delta_idx][1])


            if neighbor_cell in grid_map:
                map_update_value[delta_idx]=grid_map[neighbor_cell[0],neighbor_cell[1]][(delta_idx+2)%4]


        #updates wall map values for whatever readings are received.
        for i in range(len(readings)):
            map_update_idx=(CURRENT_DIRECTION+i-1)%4
            map_update_value[map_update_idx]=readings[i]

        #put map wall values into current position grid
        grid_map[CURRENT_POSITION[0],CURRENT_POSITION[1]]=map_update_value
        # visualize_map=collections.OrdererdDict(sorted(grid_map.items()))

        ordered_map=sorted(grid_map.items())
        print(ordered_map)


        visulize_ordered_map(ordered_map)





        # if readings[1] == 1:
        mmToInch=0.0393701
        fDistance = fSensor.get_distance()*mmToInch
        lDistance = lSensor.get_distance()*mmToInch
        rDistance = rSensor.get_distance()*mmToInch
        # fDistance=19
        if fDistance < 18:
            current_angle=utils.getIMUDegrees()
            rounded_angle=round(current_angle / 90) * 90
            if lDistance>18:
                desired_angle=rounded_angle-90
                input_angle=desired_angle-current_angle
                utils.rotateA(input_angle)
                CURRENT_DIRECTION = (CURRENT_DIRECTION - 1) % 4

            elif rDistance>18:
                desired_angle=rounded_angle+90
                input_angle=desired_angle-current_angle
                utils.rotateA(input_angle)
                CURRENT_DIRECTION = (CURRENT_DIRECTION + 1) % 4
            else:
                utils.rotateA(180)
                CURRENT_DIRECTION = (CURRENT_DIRECTION + 2) % 4
        else:
            utils.moveCell(18,2)
            CURRENT_POSITION[0]=neighbor_cells_delta[CURRENT_DIRECTION][0]+CURRENT_POSITION[0]
            CURRENT_POSITION[1]=neighbor_cells_delta[CURRENT_DIRECTION][1]+CURRENT_POSITION[1]



    print("END MAPPING HERE")
    visulize_ordered_map(ordered_map)
    time.sleep(1000000)


    # + "going" '''+ directions[CURRENT_DIRECTION]'''
    '''print("Robot is at: {} going {}".format(cell_to_coord[localized_cell], directions[CURRENT_DIRECTION]))
    closest_cell = get_closest_unvisited(visited, output_arr,max_idx[0],max_idx[1])
    print("Finished localizing closest cell to {} is {}".format(localized_cell, closest_cell))
    wavefront_arr = wavefront(closest_cell)
    path = get_path(wavefront_arr,localized_cell,closest_cell)
    print("Path to goal is {}".format(path))'''

        # print(get_readings())
if __name__ == '__main__':
    main()
