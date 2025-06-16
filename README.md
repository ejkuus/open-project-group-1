# Group 1 Open project
**Personal Drone shield**

Aleksander, Eero, Elias and Santtu

## Scenario information and the vision

Vision for the project was to create a personal drone shield. Simply putk, a Tello drone is protecting a turtlebot and flies towards a spotted threat near the turtlebot in order to keep the turtlebot safe. We chose this use case because it can be a relevant use case in many of the current political conflicts around the world.

In an example scenario, the turtlebot represents some high value carriage, truck or payload that is in in transit whilst it is approached by external threats. A use case where this kind of two way communication would be used is when the drone is used as a way to take down unfriendly drones approaching the payload. When the drone approaches the payload, the defensive drone could use e.g. an attached net to tangle and take down the unfriendly drone and return back to the payload and send a message to continue the trip.

In our simplified project we planned for the turtlebot to plan a path through a known map. The drone would then follow the bot through the planned route and patrols around it and if it detects something unfamiliar that moves, then the drone starts to fly towards the threat to intimidate it. If the threat backs off, the drone returns to patrolling around the turtlebot. As mentioned in the last section of this README, this goal was not reached mostly due to lack of time due to difficulties scheduling in course work an actual work simultaneously.

## Instructions 

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

## Discussion
**What's working and what's not**

- The drone hovers and follows the turtlebot and reacts to objects
- However, it does not react correctly to the objects and starts to fly to the wrong direction, never returning
- We found that it was challenging to implement the turtlebotv3 into gazebo, and the teleop keyboard doesn't work either
- It was nice to see that the drone would follow the bot and react to the objects
