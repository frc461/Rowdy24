# Elevator Documentation

Software implementation of the Intake Carriage subsystem on the Rowdy24 robot. The function of the intake carriage subsystem is to move both the intake and carriage in order to pick up and store a note. The intake uses a REV VORTEX and the carriage uses a REV NEO. 

## Implementation

### Variables
The intake carriage is controlled by two independent motor controllers. It has three beam breaks to track the position of a note in the robot. It also contains the lights to notify the drive team whether there is a note in the robot. 

The boolean intaking is true when the robot is intaking a note and false when it is not. 

### Methods
There are many getter methods to get the status of the various motors and sensors. There are also many setter methods to set the speeds of the motors. 