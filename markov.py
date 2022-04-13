directions = ['W', 'N', 'E', 'S']

# Motion Model
prob_forward = 0.7
prob_left = 0.1
prob_right = 0.1
prob_stay = 0.2

# Size of grid square in inches
SQ_SIZE = 18

def model_update(arr, direction):
    for row in range(len(arr)):
        for col in range(len(arr[0])):
            # Going West
            if direction == 'W':
                if col - 1 >= 0:
                    arr[row][col-1] += arr[row][col-1] * prob_forward
                if row + 1 < SQ_SIZE:
                    arr[row+1][col] += arr[row+1][col] * prob_left
                if row - 1 >= 0:
                    arr[row-1][col] += arr[row-1][col] * prob_right
                arr[row][col] += arr[row][col] * prob_stay
            # Going North
            if direction == 'N':
                if row - 1 >= 0:
                    arr[row-1][col] += arr[row-1][col] * prob_forward
                if col - 1 >= 0:
                    arr[row][col-1] += arr[row][col-1] * prob_left
                if col + 1 < SQ_SIZE:
                    arr[row][col + 1] += arr[row][col +1] * prob_right
                if row - 1 >= 0:
                    arr[row-1][col] += arr[row-1][col] * prob_right
                arr[row][col] += arr[row][col] * prob_stay

            # Going East
            # Going South


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

    # Map of walls based on cell number
    grid_map = {1:'WWOW', 2:'OWOW', 3:'OWOO', 4:'OWWO',
                5:'WWOO', 6:'OWWO', 7:'WOWO', 8:'WOWO',
                9:'WOWO', 10:'WOOW', 11:'OOWW', 12:'WOWO',
                13:'WOOW', 14:'OWOW', 15:'OWOW', 16:'OOWW'}

    surrounded_squares = [1]
    adjacent_squares = [4,5,6,10,11,13,16]
    opposite_squares = [2,7,8,9,12,14,15]
    single_square = [3]

    rows,cols = (4,4)
    arr = [[0]*cols]*rows

    NUM_SQ = rows * cols
    starting_probability = 1/NUM_SQ
    starting_direction = directions[1]
    #print(starting_possibility)
    #print(cell_num.keys())
    for i in range(len(arr)):
        for j in range(len(arr)):
            #print((i,j),cell_num.get((i,j)))
            #temp_cell_num = cell_num.get((i,j))
            #print(grid_map.get(temp_cell_num))
            arr[i][j]=starting_probability

    arr = model_update(arr,starting_direction)

    for row in range(len(arr)):
        print(arr[row])
        '''for column in range(len(arr[0])):
            #print(arr[row][column])
            pass'''

if __name__ == '__main__':
    main()