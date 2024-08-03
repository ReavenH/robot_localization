1. None -> 1: 
2. None -> 2: 
3. None -> 3: 
4. 1 -> 1: 
5. 1 -> 2: 
6. 1 -> 3: 
7. 2 -> 1: 
8. 2 -> 2: 
9. 2 -> 3: 
10. 3 -> 1: 
11. 3 -> 2: 
12. 3 -> 3: 
13. 1 -> None: 
14. 2 -> None: 
15. 3 -> None: 

| To / From | None                      | 1                   | 2                   | 3                   |
| --------- | ------------------------- | ------------------- | ------------------- | ------------------- |
| None      | Walking on the floor      | Go to fetch a brick | Go to fetch a brick | Go to fetch a brick |
| 1         | Establish the first brick | F2                  | F2                  | F2                  |
| 2         | Establish the first brick | F1 / FR1            | F1 / FR1            | F1 / FR1            |
| 3         | Establish the first brick | F1 / FL1            | F1 / FL1            | F1 / FL1            |

## Priliminary Goal

Build a straight wall with 3 layers.

## Single Robot Mapping and Navigation

Develop a compiler (simpler functionalities first, to build a wall), and verify theoretical limits using simulation.

Brick map should be discretized (grid cells with a fixed size). It should also be pre-defined.

Ups and downs on the stairs should be detected using the IMU, the height of the grid cell should be accumulated.

The building of the first layer does not rely on any ground markers, so the robot can only align by pushing (an origin brick has been provided). The first brick to be placed at each layer should rely on accurate localization since no pushing is allowed.

When generating the route to place bricks, we should consider the push direction which needs a supporting brick already  existed. and the sites being constructed should not exceed 2 layers.

Workflow reformulation: store the bricks in a grid cell map. Develop translation functions between the grid cell coordinates and the Cartesian coordinates.

### On bricks

- Walk onto the origin brick: localize itself using AprilTags, determine the brick's position using the dots on the side.
- On the origin brick: update the robot's pose at each crossroads based on the circle patterns (x, y, z positions and bearing in the grid cell and which brick it stands on).
- Navigate to the next brick according to the brickmap: pinpoint the current and desired locations, translate the route into ordered motions.

