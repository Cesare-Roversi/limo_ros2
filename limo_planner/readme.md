# TO RUN:
build workspace
source workspace
```bash
ros2 launch limo_planner launch_default.launch.py
```
and in another terminal: 
```bash
ros2 run limo_planner controller
```
### For simulations:

You can use 4 terminals in order to test if everything works. Launch in order (in different terminals):
```bash
ros2 launch limo_car robot_gazebo.launch.py
```
```bash
ros2 launch limo_bringup bringup_ackermann_amcl.launch.py
```
(or another launch file if you change controller)
```bash
ros2 launch limo_planner launch_default.launch.py
```
```bash
ros2 run limo_planner controller
```

## USEFUL FACTS FOR SIMULATIONS:
The waypoints currenly added by the controller are:
wp0,wp1,wp2 -> for the Povo simulation
wp3,wp4,wp5 -> for the Laboratory simulation

If you want to add an object to grab in the simulation with the Laboratory, use `limo_description\worlds\object\obj.obj` with theese coordinates:
(x:0.73, y:2.67, z:0.24, roll:0, pitch:0, yaw:0)

## IN ORDER TO CHANGE WORLD BETWEEN POVO AND LABORATORY:
Modify where indicated theese files:
- `limo_description\maps\mappa_povo\povo.yaml`
- `limo_description\worlds\world_povo.sdf`
- `limo_description\worlds\world_povo_resources\model.sdf`

And set the waypoint where the robot starts in `limo_planner\src\controller.cpp` to wp0 in order to work with Povo or to wp3 in order to work with Laboratory. Change also the spawn coordinates in `limo_car\launch\robot_gazebo.launch.py` and in the configuration file you are using in limo_bringup
