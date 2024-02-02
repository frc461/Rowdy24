package frc.robot.subsystems;

import com.revrobotics.CANSparkLowLevel.MotorType;
import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.SparkMaxAlternateEncoder;
import com.revrobotics.SparkPIDController;
import com.revrobotics.CANSparkBase.ControlType;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class Angler extends SubsystemBase {
    private final CANSparkMax shooterAngler;
    private final SparkPIDController anglerPID;
    private double target;

    private RelativeEncoder anglerEncoder;

    public Angler() {
        shooterAngler = new CANSparkMax(Constants.Shooter.ANGLER_ID, MotorType.kBrushless);
        //shooterAngler.restoreFactoryDefaults();
        shooterAngler.setSmartCurrentLimit(Constants.Shooter.ANGLER_CURRENT_LIMIT);
        shooterAngler.setInverted(Constants.Shooter.ANGLER_INVERT);

        anglerEncoder = shooterAngler.getAlternateEncoder(SparkMaxAlternateEncoder.Type.kQuadrature, 8192);

        anglerPID = shooterAngler.getPIDController();
        anglerPID.setP(Constants.Shooter.ANGLER_P);
        anglerPID.setI(Constants.Shooter.ANGLER_I);
        anglerPID.setD(Constants.Shooter.ANGLER_D);

        anglerPID.setFeedbackDevice(anglerEncoder);

        //anglerEncoder = shooterAngler.getAbsoluteEncoder(Type.kDutyCycle);
        //anglerEncoder.setInverted(true);
        //anglerEncoder.setPositionConversionFactor(Constants.Shooter.ANGLE_ENCODER_CONVERSION);
        //anglerEncoder.setZeroOffset();


        target = Constants.Shooter.ANGLER_UPPER_LIMIT;
    }

    public void tiltShooter(double angle) {
        // TODO: not sure if this is the correct conversion
        double angleToRot = angle / Constants.Shooter.ANGLER_ROTATION_CONSTANT;

        if (angleToRot < anglerEncoder.getPosition()
                && anglerEncoder.getPosition() < Constants.Shooter.ANGLER_LOWER_LIMIT) {
            angleToRot = Constants.Shooter.ANGLER_LOWER_LIMIT;
        } else if (angleToRot > anglerEncoder.getPosition()
                && anglerEncoder.getPosition() > Constants.Shooter.ANGLER_UPPER_LIMIT) {
            angleToRot = Constants.Shooter.ANGLER_UPPER_LIMIT;
        }
        target = angleToRot;
        anglerPID.setReference(angleToRot, ControlType.kPosition);
    }

    public void moveAngle(double movementVector) {
        if (movementVector < 0 && anglerEncoder.getPosition() < Constants.Shooter.ANGLER_LOWER_LIMIT) {
            target = Constants.Shooter.ANGLER_LOWER_LIMIT;
            holdTilt();
            return;
        } else if (movementVector > 0 && anglerEncoder.getPosition() > Constants.Shooter.ANGLER_UPPER_LIMIT) {
            target = Constants.Shooter.ANGLER_UPPER_LIMIT;
            holdTilt();
            return;
        }
        shooterAngler.set(movementVector);
        target = anglerEncoder.getPosition();
    }

    public void holdTilt() {
        target = Math.max(target, Constants.Shooter.ANGLER_LOWER_LIMIT);
        anglerPID.setReference(target, ControlType.kPosition);
    }

    public double getEncoder() {
        return anglerEncoder.getPosition();
    }
    public double getConversion(){
        return anglerEncoder.getPositionConversionFactor();
    }
}
