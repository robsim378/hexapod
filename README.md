# Hexapod
The intent of this project is to use ROS 2 to design and build a six-legged robot. The main goal is to implement the control software for the legs and have the robot be controllable with a standard game controller.

Current stage: Motion planning for individual leg movement

### Table of Contents
[Background](#background)

[Movement](#movement)

[Architecture](#architecture)

[Roadmap](#roadmap)

## Background
There are two main categories of mobile, ground-based robots: legged and wheeled. I chose a legged robot for this project as my main goal is to learn about robotics control systems, and those are fairly simple for a wheeled robot. Legged robots also have the potential to handle rough terrain better than wheeled ones, which leaves a lot of room to improve this project once the basics are done.

The main benefit of a hexapod over a bipod or quadruped legged robot is its inherent static stability. Hexapods always have at least three legs on the ground, keeping them stable at all points throughout their step. 

## Movement
This robot will be able to move laterally and rotationally simultaneously, and should be able to respond to sudden changes in input direction and magnitude. Initially it will only be able to handle flat ground, but once the basics have been implemented I intend to improve it and make it capable of walking on slopes and rough terrain.

Hexapods have three main gaits:
- **Wave gait**. One leg is raised at a time. Slow, but very stable.
- **Ripple gait**. Two legs are raised at a time. Faster, but a bit less stable.
- **Tripod gait**. Three legs are raised at a time. Fastest but least stable.

One goal for this project is to have the robot select between these three gaits depending on the target speed. For example, there is no need to use tripod gait and sacrifice some stability when moving slowly. A major challenge I expect to arise from this is responding to a change in input magnitude in the middle of a step.

## Architecture
This project uses ROS2 Humble and contains the following nodes:
- **Input Handler** *[not yet implemented]*
    - Handles receiving input and converting it into a format intelligible by the gait controller.
    - Input can come from a number of sources. Initially, it will come from a gamepad such as an Xbox controller. Eventually, it may come from an autonomous control system. Some tests will also provide input here.
- [**Gait Controller**](/src/hexapod_nodes/src/gait_controller.cpp)
    - Determines, based on the input and the current state of the legs, where to move each leg and how fast they should move.
    - One instance of this runs for the entire system and sends commands to each leg.
    - Currently not implemented beyond providing a starting position.
- [**Leg Step Controller**](/src/hexapod_nodes/src/leg_step_controller.cpp)
    - One instance of this runs for each leg and recieves commands from the Gait Controller.
    - Recieves a target position for the foot it is responsible for and plots a path for the foot to take. 
        - For legs that are on the ground and moving the robot forward, moves them in a straight line.
        - For legs that are raised and being brought foward to prepare for the next step, moves them in a curve off the ground.
    - Sends the path to be taken to the Leg Motion controller
- [**Leg Motion Controller**](/src/hexapod_nodes/src/leg_motion_controller.cpp)
    - One instance of this runs for each leg and receives commands from its Leg Step Controller
    - Receives points along the path calculated by the Leg Step Controller and handles the inverse kinematics to determine the angles of each joint to place the foot in the next point, then sends commands to the Leg Servo Controller to move the leg to that position.
- [**Leg Servo Controller**](/src/hexapod_nodes/src/leg_servo_controller.cpp)
    - One instance of this runs for each leg and receives commands from its Leg Motion Controller.
    - Receives angles for each joint in a leg and sends commands to the servo motors in its leg to move to the corresponding positions.
    - Not yet implemented as I have not purchased any hardware for the system since I want to get the core sofware done first.
- [**Leg Position Broadcaster**](/src/hexapod_nodes/src/leg_position_broadcaster.cpp)
    - One instance of this runs for each leg and receives the same commands as the Leg Servo Controller.
    - Receives angles for each joint and publishes them to the tf2 topic, which is read by RVIZ and used for visualizing the robot in simulation.

## Roadmap
- [x] Set up ROS 2 nodes and interfaces for the project
- [x] Set up URDF and tf2 broadcaster
- [ ] Leg motion planning (In progress)
- [ ] Unit testing
- [ ] Gait controller for one gait
    - [ ] Lateral movement
    - [ ] Rotational movement
    - [ ] Simultaneous lateral and rotational movement
- [ ] Gait controller for the other two gaits
- [ ] Switch between gaits depending on input magnitude
- [ ] Input handler
- [ ] Design and build physical robot
- [ ] Leg servo controller

**This is the initial goal. Everything past here is potential ways to improve the hexapod once the above core of the project is complete**
- [ ] Handle sloped ground
- [ ] Terrain mapping with LIDAR to place feet in stable positions on rough terrain
- [ ] Autonomous navigation