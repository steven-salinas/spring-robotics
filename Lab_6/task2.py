from calendar import c


directions = ['W', 'N', 'E', 'S']

# Map of walls based on cell number and facing north
grid_map = {1:'WWOW', 2:'OWOW', 3:'OWOO', 4:'OWWO',
            5:'WWOO', 6:'OWWO', 7:'WOWO', 8:'WOWO',
            9:'WOWO', 10:'WOOW', 11:'OOWW', 12:'WOWO',
            13:'WOOW', 14:'OWOW', 15:'OWOW', 16:'OOWW'}

cell_to_index = {1:(0,0),2:(0,1),3:(0,2),4:(0,3),
                5:(1,0),6:(1,1),7:(1,2),8:(1,3),
                9:(2,0),10:(2,1),11:(2,2),12:(2,3),
                13:(3,0),14:(3,1),15:(3,2),16:(3,3)}
#print(cell_to_index[1])

index_to_cell = {(0,0):1,(0,1):2,(0,2):3,(0,3):4,
                (1,0):5,(1,1):6,(1,2):7,(1,3):8,
                (2,0):9,(2,1):10,(2,2):11,(2,3):12,
                (3,0):13,(3,1):14,(3,2):15,(3,3):16}

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

    for row in arr:
        print(row)
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

        path.append((beginning_arr[-1],directions[direction]))
        beginning_arr.append(beginning)
    
    print(path)
def main():
    goal = 10
    beginnning = 3
    wavefront_arr = wavefront(goal)
    get_path(wavefront_arr,beginnning,goal)

if __name__ == '__main__':
    main()