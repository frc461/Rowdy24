package frc.robot.subsystems;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkBase.IdleMode;
import com.revrobotics.CANSparkLowLevel.MotorType;

import edu.wpi.first.wpilibj.AddressableLED;
import edu.wpi.first.wpilibj.AddressableLEDBuffer;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.Relay;
import edu.wpi.first.wpilibj.Relay.Direction;
import edu.wpi.first.wpilibj.Relay.Value;
import edu.wpi.first.wpilibj.motorcontrol.Spark;
import edu.wpi.first.wpilibj.util.Color;

import com.revrobotics.CANSparkFlex;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class IntakeCarriage extends SubsystemBase {
    private final CANSparkFlex intake;
    private final CANSparkMax carriage;
    private final DigitalInput carriageBeam; // end of carriage (on shooter side)
    private final DigitalInput ampBeam; // entrance of carriage (which is the amp shooter)
    private final DigitalInput shooterBeam; // completely exit through shooter
    private final AddressableLED led;
    private final Spark lights;
    private final AddressableLEDBuffer buffer;
    Relay intakeLights;
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

        intakeLights = new Relay(0);
        intakeLights.setDirection(Direction.kForward);

        led = new AddressableLED(2);
        buffer = new AddressableLEDBuffer(12);
        led.setLength(buffer.getLength());
        configureLights(false);

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
            intakeLights.set(Value.kOn);
        } else {
            intakeLights.set(Value.kOff);
        }
        configureLights(noteInShootingSystem());
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

    public void configureLights(boolean on) {
        if (on) {
            for (int i = 0; i < buffer.getLength(); i++) {
                buffer.setLED(i, Color.kOrange);
            }
        } else {
            for (int i = 0; i < buffer.getLength(); i++) {
                buffer.setRGB(i, 0, 0, 0);
            }
        }
        led.setData(buffer);
        led.start();
    }
}