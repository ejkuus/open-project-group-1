# Group 1 Open project
## Personal Drone shield

Aleksander, Eero, Elias and Santtu

**Installing turtlebot:**

- Install Turtlebot v3: https://emanual.robotis.com/docs/en/platform/turtlebot3/quick-start/#pc-setup
    - Following this creates the /turtlebot3_ws directory
    - Edit the model.sdf file to the one found in root of the project (Path to file: turtlebot3_ws/src/turtlebot3_simulations/turtlebot3_gazebo/models/turtlebot3_burger/model.sdf)

**How to run the drone shield:**

- Open 4 Terminals:
- *Terminal 1:*
    - $ cd ~/finalProject/drone_racing_ros2-main$
    - $ ros2 launch tello_gazebo simple_launch.py -> This will open gazebo simulation with tello drone

 - *Terminal 2:*
    - $ cd ~/finalProject/drone_racing_ros2-main$
    - $ ros2 service call /drone1/tello_action tello_msgs/TelloAction "{cmd: 'takeoff'}" -> This will launch the drone into the air

 - *Terminal 3:*
    - $ cd ~/turtlebot3_ws
    - $ ros2 launch turtlebot3_gazebo empty_world.launch.py -> This will insert the turtlebot into the open gazebo world

 - *Terminal 4:*
    - $ cd ~/finalProject/drone_racing_ros2-main$
    - $ python3 tello_ros/tello_gazebo/scripts/follow_turtlebot_circle.py -> This will activate the *drone shield* and the drone will now follow the turtlebot anywhere it goes, circulating it at the same time


- Insert an object you like from the tello_gazebo package
- Now when the turtlebot gets close enough on the object, the drone will fly towards it, intimidating the object and then flying back to turtlebot.

![drone](https://github.com/ejkuus/open-project-group-1/blob/main/IMG_0620.gif)
