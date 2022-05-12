# Lab_6 - Mapping and Planning

- Task 1: Create a function that uses tof sensors to identify and map the internal walls of an unknown maze. 
- Task 2: Create a function to find and navigate the shortest path between a start and goal cells in a given maze, using the wavefront planner algorithm. 

---

Adjustments: 
- Task 1:
1.  Change `CURRENT_DIRECTION`*(line 28)* from `1` to another value to change the starting direction of the robot: 0 is West, 1 is North, 2 is East, 3 is South. 

- Task 2:
1.  Change `CURRENT_DIRECTION`*(line 14)* from `1` to another value to change the starting direction of the robot: 0 is West, 1 is North, 2 is East, 3 is South. Direction is based on looking down at the map created in adjustment *2*. 
2.  Change the `grid_map` *(line 37)* to adjust the layout of the walls within the map. This map a 4x4 grid creating 16 cells, each with their walls listed as `W` or `O` representing *open/wall*. The walls are formatted in the order of West, North, East, South. This is useful for indexing to access direction 0-3 respectively. 
3.  Change `beginning` cell *line 190* to change the cell to begin the pathfinding to the end cell. Cell numbers correspond to those created in `grid_map`
4.  Change `goal` cell *(line 191)* to end the pathfinding at a different cell. 

 