Folder for scripts to run in the drone.

The codes outside the drone_control folder are the ones developed for PX4 simulator:

- offb_node.py is for a simple hover based on */mavros/setpoint_position/local* with a constant publishing in this topic with only a z-coordinate to hover. This code is presented by Pixhawk in their website: https://docs.px4.io/main/en/ros/mavros_offboard_python.html
- waypoints.py is a node for the drone to travel through some waypoints after hovering. The trajectory programmed is for a square and coming back to home position. The logic os this code is as follows:

![imagen](https://github.com/RoboticaInteligente8voTecCEM2023/drone_navigation/assets/67598380/a5925eef-d225-4834-8ff8-d8daaa189539)

