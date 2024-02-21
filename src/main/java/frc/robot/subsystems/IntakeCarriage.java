package frc.robot.subsystems;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkLowLevel.MotorType;
import edu.wpi.first.wpilibj.DigitalInput;

import com.revrobotics.CANSparkFlex;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.constants.Constants;
import frc.robot.constants.RobotConstants;
import frc.robot.constants.RobotIdentity;

public class IntakeCarriage extends SubsystemBase {
    private final CANSparkFlex intake;
    private final CANSparkMax carriage;

    DigitalInput carriageBeam = new DigitalInput(6); // end of carriage (on shooter side)

    DigitalInput ampBeam = new DigitalInput(2); // entrance of carriage (which is the amp shooter)
    DigitalInput shooterBeam = new DigitalInput(7); // completely exit through shooter

    public IntakeCarriage() {
        intake = new CANSparkFlex(Constants.IntakeCarriage.INTAKE_ID, MotorType.kBrushless);
        intake.restoreFactoryDefaults();
        intake.setSmartCurrentLimit(80);
        intake.setInverted(false);

        carriage = new CANSparkMax(Constants.IntakeCarriage.CARRIAGE_ID, MotorType.kBrushless);
        carriage.restoreFactoryDefaults();
        carriage.setInverted(true);
    }

    public double getIntakeSpeed() {
        return intake.get();
    }

    public double getCarriageSpeed() {
        return carriage.get();
    }

    public boolean getAmpBeamBroken() {
        return ampBeam.get();
    }

    public boolean getShooterBeamBroken() {
        return !shooterBeam.get();
    }

    public boolean getCarriageBeamBroken() {
        return !carriageBeam.get();
    }

    public boolean noteInSystem() {
        return getShooterBeamBroken() || getCarriageBeamBroken();
    }

    public void overrideIntakeSpeed(double speed) {
        intake.set(speed);
    }

    public void overrideCarriageSpeed(double speed) {
        carriage.set(speed);
    }

    public void setIntakeCarriageSpeed(double intakeSpeed, double carriageSpeed) {
        intake.set(intakeSpeed);
        carriage.set(carriageSpeed);
    }

    public void setIntakeCarriageSpeed(double speed){
        setIntakeCarriageSpeed(speed, speed);
    }

    public void setIntakeIdle(boolean idleMode) {
        intake.set(idleMode ? Constants.IntakeCarriage.IDLE_INTAKE_SPEED : 0);
    }

    public void setCarriageIdle() {
        carriage.set(0);
    }
}