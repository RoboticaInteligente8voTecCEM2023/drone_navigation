### Camera-based navigation
This foler contains all nodes develped in Hector quadrotor simulator and the solution for the physical drone.

The files for the running the solution are:
- follow_object_full.py
- guided_controller.py
- obstacle_final.world
- obstacle_full.launch
- sk450_px4_quad_release.tar.gz

Mostly you can run the solution through the obstacle_full.launch in terminal.
sk450_px4_quad_release.tar.gz is the compressed file for gazebo simulation.

This solution is based on color object reactions: follows green objects, avoids red objects and lands when only yellow of certain area is detected. In real life implementation we changed colors to purple as green, and maintained the rest.

![imagen](https://github.com/RoboticaInteligente8voTecCEM2023/drone_navigation/assets/67598380/47a064b5-637e-4c57-af79-785f29694590)

Color detection is done with OpenCV using HSV masks per color and some morphological operations to reduce noise. Find contours afterwards and locate object centroid. Control is programmed based on the centroid, by comparing camera fixed center vs object centroid. Within the drone camera context, x-axis is left-right movement, z-axis is up-down movement, and y-axis is close-far from object.

Control node is done based on pixel location of centroid, for example object centroid at (150,50) vs camera center (100,100) means object has been detected at the right-upper sector against the camera center considering (0,0) the left-up corner of image. If the object is green, then drone should be moved towards the right and up to try encounter camera center with centroid. On the other hand if the object is red, the reference to move towards is where these is more red-free area in image in this case left-down sector. Y-axis movement is based on the objects area, meaning if the area is smaller than desired, move towards it, otherwise if the object is too close (area is bigger then desired), move back.

If there are no red or green objects, then no movement is done, and if a minimum area yellow object appears, the drone lands.

For avoiding obstacles:

![imagen](https://github.com/RoboticaInteligente8voTecCEM2023/drone_navigation/assets/67598380/e1d95e9d-ddc5-43ba-a3f6-4b06316fdc63)

For following objects:

![imagen](https://github.com/RoboticaInteligente8voTecCEM2023/drone_navigation/assets/67598380/4a70a875-7d6f-4a12-90b9-f2fd6bbc2e9f)

### Teleoperation and lidar-based navigation
The rest of files are for specific nodes:
- follow_wall.py: the methods for following wall using left or right hand rule.
- rhr_drone.py: follows wall using right hand rule based on the hector drone lidar.

Based on these two files, right hand rule is coded based on trigonometry and lidar sector information for getting a two-parts controller: angle correction (to move parallel to wall) and distance correction (to move at an specific distance from wall)
![imagen](https://github.com/RoboticaInteligente8voTecCEM2023/drone_navigation/assets/67598380/29b9aa85-b0e7-464f-bb3d-4de844e02ca2)

![imagen](https://github.com/RoboticaInteligente8voTecCEM2023/drone_navigation/assets/67598380/ab995a0f-12b4-43a2-8f6e-2feddaa53ddc)

![imagen](https://github.com/RoboticaInteligente8voTecCEM2023/drone_navigation/assets/67598380/8c2516df-5391-420c-b762-f17ab3063fcb)

- hover.py: controlls drone with joystick/xbox controller. You must have a compatible controller. Read the installation/configuration guide at the /info section of this repository
