# **CS489 final draft project**

## Plan

### Part 1 Navigate the maze

- Mark entrance as a wall in occupancy grid
  <strike>- dead (DFS) Move until intersection -> go in first direction we see (mark visited cells)</strike>
  - We will detect an intersection by scanning left and right by some amount (determined through testing) and if we detect free space then we will add that direction into out stack.
- Once we reach a deadend -> backtrack to the last intersection on the stack -> explore unexplored directions.
- If we find exit (distance we see > max distance) mark as exit, but keep exploring if we have not explored the whole maze.

### Part 2 Avoid moving obstacles

- If the obstacle takes up a certain amount of space on our camera (red pixels). Pause movement.

### Part 3 April Tags

- Hashmap (this will ensure no duplicates)
- Key: april tag code
- Value: coordinates

### Questions

- Map layout

## Timeline

### Week 11

- Ask questions
- Complete final plan

### Week 12

- Scaffold project including all nodes and methods
- Implement maze navigation algorithm

### Week 13

- Implement obstacle avoidance
- implement april tag scanning

### Week 14 (check in to see if we have implemented all core functionality)

- debug

## Splitting up the workload

- We will all meet together in person during class and robot hours and have someone drive.
