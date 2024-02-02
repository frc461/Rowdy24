package frc.robot.subsystems;

import com.revrobotics.CANSparkLowLevel.MotorType;
import com.revrobotics.CANSparkMax;
import com.revrobotics.SparkAbsoluteEncoder;
import com.revrobotics.SparkAbsoluteEncoder.Type;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class Angler extends SubsystemBase {
    private final CANSparkMax shooterAngler;
    private final PIDController anglerPID;
    private final SparkAbsoluteEncoder anglerEncoder;
    private double target;

    public Angler() {
        shooterAngler = new CANSparkMax(Constants.Shooter.ANGLER_ID, MotorType.kBrushless);
        //shooterAngler.restoreFactoryDefaults();
        shooterAngler.setSmartCurrentLimit(Constants.Shooter.ANGLER_CURRENT_LIMIT);
        shooterAngler.setInverted(Constants.Shooter.ANGLER_INVERT);
        anglerPID = new PIDController(
                Constants.Shooter.ANGLER_P,
                Constants.Shooter.ANGLER_I,
                Constants.Shooter.ANGLER_D
        );
        anglerEncoder = shooterAngler.getAbsoluteEncoder(Type.kDutyCycle);
        //anglerEncoder.setInverted(true);
        anglerEncoder.setPositionConversionFactor(Constants.Shooter.ANGLE_ENCODER_CONVERSION);
        target = Constants.Shooter.ANGLER_UPPER_LIMIT;
    }

    public void tiltShooter(double angle) {
        // TODO: not sure if this is the correct conversion
        double angleToRot = angle; //* Constants.Shooter.ANGLER_ROTATION_CONSTANT;

        if (angleToRot < anglerEncoder.getPosition()
                && anglerEncoder.getPosition() < Constants.Shooter.ANGLER_LOWER_LIMIT) {
            angleToRot = Constants.Shooter.ANGLER_LOWER_LIMIT;
        } else if (angleToRot > anglerEncoder.getPosition()
                && anglerEncoder.getPosition() > Constants.Shooter.ANGLER_UPPER_LIMIT) {
            angleToRot = Constants.Shooter.ANGLER_UPPER_LIMIT;
        }
        target = angleToRot;
        shooterAngler.set(anglerPID.calculate(anglerEncoder.getPosition(), angleToRot));
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
        shooterAngler.set(anglerPID.calculate(anglerEncoder.getPosition(), target));
    }

    public double getEncoder() {
        return anglerEncoder.getPosition();
    }
}
