# Angler Documentation

Software implementation of the angler subsystem on the Rowdy24 robot. The function of the angler subsystem is to raise or lower the angle of the shooter to aim for the speaker target and prepare to shoot a note. Its motor is a Rev NEO 550.

## Implementation

### Class Information

The `Angler` class extends the [`SubsystemBase`](https://github.wpilib.org/allwpilib/docs/release/java/edu/wpi/first/wpilibj2/command/SubsystemBase.html) class, which helps to register commands involving the subsystem.

### Variables

The angler is entirely controlled by a [motor controller](../../src/main/java/frc/robot/subsystems/Angler.java#L19), which is initially defined. Its components that are useful include:
- [Encoder](../../src/main/java/frc/robot/subsystems/Angler.java#L22) - the motor's encoder is what converts electrical signals into implementable data values.
- [Integrated PID Controller](../../src/main/java/frc/robot/subsystems/Angler.java#L25) - the motor's PID controller uses the feedback from the software implementation of target and the encoded position to directly change the electrical output of the motor.

Additionally, an [external limit switch](../../src/main/java/frc/robot/subsystems/Angler.java#L28) is also positioned to re-zero the angler's encoded value at the bottom mechanical limit. Re-zeroing calibrates the encoded position with the actual position of the angler.

Numerical metrics include [`target`](../../src/main/java/frc/robot/subsystems/Angler.java#L31), [`error`](../../src/main/java/frc/robot/subsystems/Angler.java#L37), and [`accuracy`](../../src/main/java/frc/robot/subsystems/Angler.java#L43). `target` represents the encoded value that the motor needs to move the angler position to through electric output. `error` and `accuracy` determine how close the angler actually is to where it is supposed to be, considering possible input values to move or set the angler to a certain position.

### Methods

Many getter functions are defined in the class for utility. Additionally, a `periodic` function for the subsystem is used to update the `accuracy` and `error` variables.

The most useful methods are:
- [`holdTarget()`](../../src/main/java/frc/robot/subsystems/Angler.java#L160) sets the PID reference to the target variable.
- [`moveAngle(double AxisValue)`](../../src/main/java/frc/robot/subsystems/Angler.java#L171) gives the angler power based on the joystick value from the operator.
- [`setEncoderVal(double encoderVal)`](../../src/main/java/frc/robot/subsystems/Angler.java#L191) sets the encoder value to the specified `encoderVal` value and remains in that position.