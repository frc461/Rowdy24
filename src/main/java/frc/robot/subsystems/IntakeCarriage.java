package frc.robot.subsystems;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkLowLevel.MotorType;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

import com.revrobotics.CANSparkFlex;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Configuration;
import frc.robot.RobotConstants;
import frc.robot.RobotIdentity;

public class IntakeCarriage extends SubsystemBase {
    private final CANSparkFlex intake;
    private final CANSparkMax carriage;
    private final RobotConstants robot = RobotConstants.getRobotConstants(RobotIdentity.getIdentity());
    private final Configuration configuration = robot.getConfiguration();

    DigitalInput carriageBeam = new DigitalInput(6); // end of carriage (on shooter side)

    DigitalInput ampBeam = new DigitalInput(2); // entrance of carriage (which is the amp shooter)
    DigitalInput shooterBeam = new DigitalInput(7); // completely exit through shooter

    public IntakeCarriage() {
        intake = new CANSparkFlex(configuration.intake_id, MotorType.kBrushless);
        intake.restoreFactoryDefaults();
        intake.setSmartCurrentLimit(80);
        intake.setInverted(false);

        carriage = new CANSparkMax(configuration.carriage_id, MotorType.kBrushless);
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
        intake.set(idleMode ? configuration.idle_intake_speed : 0);
    }

    public void setCarriageIdle() {
        carriage.set(0);
    }
}