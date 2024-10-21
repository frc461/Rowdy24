Package frc.robot.subsystems
# Class Angler

java.lang.Object 
<br> &nbsp;&nbsp;&nbsp;&nbsp;edu.wpi.first.wpilibj2.command.SubsystemBase
<br> &nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;frc.robot.subsystems.Angler

<b>All Implemented Interfaces:</b><br>`Sendable, Subsystem`<hr>
```
public class Angler
extends SubsystemBase
```

Software implementation of the Angler subsystem on the Rowdy24 robot. The function of the angler subsystem is to raise or lower the angle of the shooter to aim for the speaker target and prepare to shoot a note. The motor controller is a Rev NEO 550.

## Field Summary

| **Modifier and Type** |   | **Field**              |   | **Description**                                                              |
|-----------------------|---|------------------------|---|------------------------------------------------------------------------------|
| CANSparkMax           |   | angler                 |   | Represents the motor controller of the angler.                               |
| RelativeEncoder       |   | encoder                |   | The motor's encoder.                                                         |
| SparkPIDController    |   | anglerPIDController    |   | The motor's integrated PID controller.                                       |
| SparkLimitSwitch      |   | lowerMagnetLimitSwitch |   | **Unused** The motor's integrated lower limit switch.                        |
| SparkLimitSwitch      |   | upperMagnetLimitSwitch |   | **Unused** The motor's integrated upper limit switch                         |
| DigitalInput          |   | lowerLimitSwitch       |   | Represents a limit switch defined as the lower limit of the angler.          |
| double                |   | target                 |   | The target state of the encoder position.                                    |
| double                |   | error                  |   | The difference between target state and current encoder position.            |
| double                |   | accuracy               |   | The percentage similarity between target state and current encoder position. |

## Constructor Summary

