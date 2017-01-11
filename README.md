# RoboCops
Praktikum Mobile Roboter 64-156, Wintersemester 2016/2017

## Strategies

### Random Walk
- Different points on map.
- Choose the closest point from all the points which are not yet been visited.
- Move arround between points.

### Optimize Shooting
1. Detect an AprilTag.
2. Calculate absolute Optimal Shooting Position (OSP) on map.
    - Transform the AprilTag position from camera coordinate to robot_base one.
    - Calculate the best position (z=1, x=0, ori=?) on robot_base coordinate.
    - Transform the best position from robot_base coordinate to camera one.
3. Move to this OSP.
4. The target might move. Just follow.

![transformation](http://i.imgur.com/HCwK6HX.jpg)

### After Getting Shot
1. 5 seconds freezed
    - Scan surrounding for moving objects.
    - Transform laser scan data to map.
2. After 5 seconds
    - Move to optimal shooting position.

### After Shooting (Hide & Attack)
1. Calculate the potential position (reachable in 5 secs) of the opponent robot and save it as area of interest (AOI).
2. Move backwards out of AOI.
3. Move forward and search the AOI to attach again.
