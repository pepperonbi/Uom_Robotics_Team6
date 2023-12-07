# Uom_Robotics_Team6

This is a code repository for our robot.

## Intruduction

This code repository contains the necessary code required to implement the robot's functionality, which includes the code of core robot functions, sensors, manipulater, etc. For the ultimate refinement of the project, this document may also be improved as the task progresses.

## Description of Robot

The robot will be capable of retrieving bright-coloured wooden cubes from an environment. The operation is set in an environment characterised by dynamic lighting conditions and the presence of static obstacles that necessitate obstacle detection and avoidance. The robot is required to retrieve all objects within a 30-minute time frame, while maintaining a high level of au- tonomy and adhering to safety standards.

## System Structure

The firmware operates directly on the LeoCore board's processor. It offers various functionalities to the Raspberry Pi via a serial bus. Additionally, it wirelessly connects to an NUC, which oversees sensors (depth camera and LiDAR) and the manipulator. Simultaneously, the Raspberry Pi houses a set of ROS nodes that enable access to various features through ROS topics and services.

(Insert a diagram at this place. )

## Reference

[Leo Rover v.1.8 Official Documentation](https://www.leorover.tech/knowledge-base)
