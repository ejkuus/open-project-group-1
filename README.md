# Group 1 Open project
## Personal Drone shield

Aleksander, Eero, Elias and Santtu

**How to run:**

- Install Turtlebot v3: https://emanual.robotis.com/docs/en/platform/turtlebot3/quick-start/#pc-setup
- Open 4 Terminals:
- *Terminal 1:*
    - $ cd ~/finalProject/drone_racing_ros2-main$
    - $ ros2 launch tello_gazebo simple_launch.py' # This will open gazebo simulation with tello drone

 - *Terminal 2:*
    - $ cd ~/turtlebot3_ws$
    - $ ros2 launch turtlebot3_gazebo empty_world.launch.py # This will inster the turtlebot into the open gazebo world

 - *Terminal 3:*
    - $ cd ~/finalProject/drone_racing_ros2-main$
    - $ ros2 service call /drone1/tello_action tello_msgs/TelloAction "{cmd: 'takeoff'}" # This will launch the drone into the air
  
 - *Terminal 4:*
    - $ cd ~/finalProject/drone_racing_ros2-main$
    - $ python3 tello_ros/tello_gazebo/scripts/follow_turtlebot_circle.py # This will activate the *drone shield* and the drone will now follow the turtlebot anywhere it goes, circulating it at the same time
