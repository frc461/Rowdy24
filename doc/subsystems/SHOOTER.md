# Shooter Documentation

Software implementation of the Shooter subsystem on the Rowdy24 robot. The function of the shooter subsystem is to spin the shooter wheels to lauch a note towards the speaker. There are two REV Vortexes. One is for the top set of wheels and the other is for the bottom.

## Implementation

### Variables
The shooter is controlled by two motor controllers, which each has an encoder and a PID controller.  

Numerical metrics include target, error, and accuracy. Target represents the speed that the motor needs move the shooter at through electric output. Error and accuracy determine how close the shooter actually is to where it is supposed to be, considering possible input values to move or set the shooter to a certain speed.

### Methods
There are getter and setter methods to get information from the motors and set them to a certain speed. 