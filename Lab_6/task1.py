import time
import signal
import matplotlib.pyplot as plt

import PID
import RPi.GPIO as GPIO # README: https://pypi.org/project/RPi.GPIO/
import utils

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



# 0 = west, 1 = north, 2 = east, 3 = south
CURRENT_DIRECTION = 1


def get_readings():
    """
    Gets the readings from the left, front, and right sensor respectively
    A '1' repsresents a wall while a '0' represents no wall
    Output - List[left:int,front:int,right:int]
    """

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
    """
    Main function of Lab_6/task1.py
    This function explores a maze and visualizes the output using mathplotlib.pyplot
    """

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

    grid_map={}
    start_position=[0,0]
    CURRENT_POSITION=start_position
    neighbor_cells_delta=((-1,0),(0,1),(1,0),(0,-1))

    # Robot will continue exploring until the amount of explored cells is 16
    while len(grid_map)<16:
        readings = get_readings()

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
        ordered_map=sorted(grid_map.items())
        print(ordered_map)

        visulize_ordered_map(ordered_map)

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

if __name__ == '__main__':
    main()
