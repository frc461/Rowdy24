package frc.robot.subsystems;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkBase.IdleMode;
import com.revrobotics.CANSparkLowLevel.MotorType;
import edu.wpi.first.hal.RelayJNI;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.motorcontrol.Spark;
import com.revrobotics.CANSparkFlex;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class IntakeCarriage extends SubsystemBase {
    private final CANSparkFlex intake;
    private final CANSparkMax carriage;
    private final DigitalInput carriageBeam; // end of carriage (on shooter side)
    private final DigitalInput ampBeam; // entrance of carriage (which is the amp shooter)
    private final DigitalInput shooterBeam; // completely exit through shooter
    private final Spark lights;
    private boolean intaking;

    public IntakeCarriage() {
        intake = new CANSparkFlex(Constants.IntakeCarriage.INTAKE_ID, MotorType.kBrushless);
        intake.restoreFactoryDefaults();
        intake.setSmartCurrentLimit(80);
        intake.setInverted(false);
        intake.setIdleMode(IdleMode.kCoast);

        carriage = new CANSparkMax(Constants.IntakeCarriage.CARRIAGE_ID, MotorType.kBrushless);
        carriage.restoreFactoryDefaults();
        carriage.setInverted(true);

        carriageBeam = new DigitalInput(Constants.IntakeCarriage.CARRIAGE_BEAM);
        ampBeam = new DigitalInput(Constants.IntakeCarriage.AMP_BEAM);
        shooterBeam = new DigitalInput(Constants.IntakeCarriage.SHOOTER_BEAM);

        RelayJNI.initializeRelayPort(0, true);
        lights = new Spark(Constants.IntakeCarriage.LIGHT_ID);

        intaking = false;
    }

    @Override
    public void periodic() {
        if (noteInShootingSystem()) {
            lights.set(0.77);
        } else {
            lights.set(0.61);
        }
        if (intaking) {
            RelayJNI.setRelay(0, true);
        } else {
            RelayJNI.setRelay(0, false);
        }
    }

    public double getIntakeSpeed() {
        return intake.get();
    }

    public double getCarriageSpeed() {
        return carriage.get();
    }

    public boolean getAmpBeamBroken() {
        return !ampBeam.get();
    }

    public boolean getShooterBeamBroken() {
        return !shooterBeam.get();
    }

    public boolean getCarriageBeamBroken() {
        return !carriageBeam.get();
    }

    public boolean noteInAmpSystem() {
        return getCarriageBeamBroken() || getAmpBeamBroken();
    }

    public boolean noteInShootingSystem() {
        return getShooterBeamBroken() || getCarriageBeamBroken();
    }

    public void setIntaking(boolean intaking) {
        this.intaking = intaking;
    }

    public void setIntakeSpeed(double speed) {
        intake.set(speed);
    }

    public void setCarriageSpeed(double speed) {
        carriage.set(speed);
    }

    public void setIntakeCarriageSpeed(double intakeSpeed, double carriageSpeed) {
        setIntakeSpeed(intakeSpeed);
        setCarriageSpeed(carriageSpeed);
    }

    public void setIntakeCarriageSpeed(double speed){
        setIntakeCarriageSpeed(speed, speed);
    }
}