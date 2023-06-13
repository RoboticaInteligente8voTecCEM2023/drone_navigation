This foler contains all nodes develped in Hector quadrotor simulator and the solution for the physical drone.

The files for the running the solution are:
- follow_object_full.py
- guided_controller.py
- obstacle_final.world
- obstacle_full.launch
- sk450_px4_quad_release.tar.gz

Mostly you can run the solution through the obstacle_full.launch in terminal.
sk450_px4_quad_release.tar.gz is the compressed file for gazebo simulation.

The rest of files are for specific nodes:
- follow_wall.py: the methods for following wall using left or right hand rule.
- rhr_drone.py: follows wall using right hand rule based on the hector drone lidar.
- hover.py: controlls drone with joystick/xbox controller
