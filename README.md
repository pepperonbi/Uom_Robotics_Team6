# Uom_Robotics_Team6

This is a code repository for the robot.

## Introduction

This code repository contains the necessary code required to implement the robot's functionality, which includes the code of core robot functions, sensors, manipulator, etc. For the ultimate refinement of the project, this document may also be improved as the task progresses.

## Description of Robot

The robot will be capable of retrieving bright-coloured wooden cubes from an environment. The operation is set in an environment characterised by dynamic lighting conditions and the presence of static obstacles that necessitate obstacle detection and avoidance. The robot is required to retrieve all objects within a 30-minute time frame, while maintaining a high level of autonomy and adhering to safety standards.

## System Structure

The firmware operates directly on the LeoCore board's processor. It offers various functionalities to the Raspberry Pi via a serial bus. Additionally, it wirelessly connects to an NUC, which oversees sensors (depth camera and LiDAR) and the manipulator. Simultaneously, the Raspberry Pi houses a set of ROS nodes that enable access to various features through ROS topics and services.

![System Structure](/Graph/System%20Diagram%20of%20the%20Leo%20Rover%20and%20NUC.jpeg)

## Repository Structure

The tree shows a rough stucture of this repository, including important packages and directories.

```
Uom_Robotics_Team6
├── Graph                        # Graphs or pictures for introduction
├── LINK FOR CAD.txt             
├── README.md
└── team06_rover_ws              # The workspace of Team 6
    └── src                      
        ├── auto_nav_pkg         # Package for navigation and mapping
        │   ├── auto_nav_pkg
        │   ├── map
        │   └── launch
        ├── leo_description      # Model of Leo Rover
        └── object_det_pkg       # Package for object detection and grasping
            ├── object_det_pkg       
            └── launch

```

## Contributors

Jorge Corpa Chung: jorge.corpachung@student.manchester.ac.uk

Praneel Raghuraman: praneel.Raghuraman@postgrad.manchester.ac.uk

Moyan Zhang: moyan.zhang@postgrad.manchester.ac.uk

Zhirui Zhang: zhirui.zhang@postgrad.manchester.ac.uk

## Reference

[Leo Rover v.1.8 Official Documentation](https://www.leorover.tech/knowledge-base)
