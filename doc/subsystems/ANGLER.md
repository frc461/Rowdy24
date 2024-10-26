# Angler Documentation

Software implementation of the Angler subsystem on the Rowdy24 robot. The function of the angler subsystem is to raise or lower the angle of the shooter to aim for the speaker target and prepare to shoot a note. The motor is a Rev NEO 550.

## Implementation

### Class Information

The angler class extends the [SubsystemBase](https://github.wpilib.org/allwpilib/docs/release/java/edu/wpi/first/wpilibj2/command/SubsystemBase.html) class, which helps to register commands involving the subsystem.

### Variables

The angler is entirely controlled by a motor controller, which is initially defined. Its components that are useful include:
- Encoder - the motor's encoder is what converts electrical signals into implementable data values.
- Integrated PID Controller - the motor's PID controller uses the feedback from the software implementation of target and the encoded position to directly change the electrical output of the motor.

Additionally, an external limit switch is also installed to re-zero the angler's encoded value at the bottom mechanical limit. Re-zeroing calibrates the encoded position with the actual position of the angler.

Numerical metrics include target, error, and accuracy. Target represents the encoded value that the motor needs to move the angler position to through electric output. Error and accuracy determine how close the angler actually is to where it is supposed to be, considering possible input values to move or set the angler to a certain position.

### Methods

Many getter functions are defined in the class for utility.

