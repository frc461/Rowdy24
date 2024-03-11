package frc.robot.subsystems;

import com.revrobotics.CANSparkBase.ControlType;
import com.revrobotics.CANSparkFlex;
import com.revrobotics.CANSparkLowLevel.MotorType;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.SparkPIDController;
import edu.wpi.first.units.*;
import edu.wpi.first.wpilibj.RobotController;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine;
import frc.robot.Constants;

import static edu.wpi.first.units.MutableMeasure.mutable;
import static edu.wpi.first.units.Units.*;

public class Shooter extends SubsystemBase {
    private final CANSparkFlex bottomShooter, topShooter;

    private final SparkPIDController bottomController, topController;

    private final RelativeEncoder bottomEncoder, topEncoder;
    private final MutableMeasure<Voltage> appliedVoltage = mutable(Volts.of(0));
    private final MutableMeasure<Angle> angle = mutable(Rotations.of(0));
    private final MutableMeasure<Velocity<Angle>> velocity = mutable(RPM.of(0));
    private final SysIdRoutine sysIdRoutine;
    private double target, error, accuracy;

    public Shooter() {
        bottomShooter = new CANSparkFlex(Constants.Shooter.BOTTOM_SHOOTER_ID, MotorType.kBrushless);
        bottomShooter.restoreFactoryDefaults();
        bottomShooter.setSmartCurrentLimit(Constants.Shooter.SHOOTER_CURRENT_LIMIT);
        bottomShooter.setInverted(!Constants.Shooter.SHOOTER_INVERT);
        bottomEncoder = bottomShooter.getEncoder();

        topShooter = new CANSparkFlex(Constants.Shooter.TOP_SHOOTER_ID, MotorType.kBrushless);
        topShooter.restoreFactoryDefaults();
        topShooter.setSmartCurrentLimit(Constants.Shooter.SHOOTER_CURRENT_LIMIT);
        topShooter.setInverted(!Constants.Shooter.SHOOTER_INVERT);
        topEncoder = topShooter.getEncoder();

        bottomController = bottomShooter.getPIDController();
        topController = topShooter.getPIDController();

        bottomController.setP(Constants.Shooter.SHOOTER_P);
        bottomController.setI(Constants.Shooter.SHOOTER_I);
        bottomController.setD(Constants.Shooter.SHOOTER_D);
        bottomController.setFF(Constants.Shooter.SHOOTER_FF);

        topController.setP(Constants.Shooter.SHOOTER_P);
        topController.setI(Constants.Shooter.SHOOTER_I);
        topController.setD(Constants.Shooter.SHOOTER_D);
        topController.setFF(Constants.Shooter.SHOOTER_FF);

        bottomShooter.burnFlash();
        topShooter.burnFlash();

        sysIdRoutine = new SysIdRoutine(
                new SysIdRoutine.Config(),
                new SysIdRoutine.Mechanism(
                        (Measure<Voltage> volts) -> {
                            topShooter.setVoltage(volts.in(Volts));
                            bottomShooter.setVoltage(volts.in(Volts));
                        },
                        log -> {
                            log.motor("topShooter")
                                    .voltage(
                                            appliedVoltage.mut_replace(
                                                    topShooter.get() * RobotController.getBatteryVoltage(), Volts
                                            ))
                                    .angularPosition(
                                            angle.mut_replace(
                                                    topEncoder.getPosition(), Rotations
                                            ))
                                    .angularVelocity(
                                            velocity.mut_replace(
                                                    topEncoder.getVelocity(), RPM
                                            ));
                            log.motor("bottomShooter")
                                    .voltage(
                                            appliedVoltage.mut_replace(
                                                    bottomShooter.get() * RobotController.getBatteryVoltage(), Volts
                                            ))
                                    .angularPosition(
                                            angle.mut_replace(
                                                    bottomEncoder.getPosition(), Rotations
                                            ))
                                    .angularVelocity(
                                            velocity.mut_replace(
                                                    bottomEncoder.getVelocity(), RPM
                                            ));

                        },
                        this
                )
        );

        target = 0.0;
        error = 0.0;
        accuracy = 1.0;
    }

    @Override
    public void periodic() {
        error = Math.abs(target - (getBottomShooterSpeed() + getTopShooterSpeed()) / 2);
        accuracy = (target > (getBottomShooterSpeed() + getTopShooterSpeed()) / 2) ?
                ((getBottomShooterSpeed() + getTopShooterSpeed()) / 2) / target :
                target / ((getBottomShooterSpeed() + getTopShooterSpeed()) / 2);
    }

    public double getBottomShooterSpeed() {
        return bottomEncoder.getVelocity();
    }

    public double getTopShooterSpeed() {
        return topEncoder.getVelocity();
    }

    public double getError() {
        return error;
    }

    public void shoot(double speed) {
        target = speed;
        topController.setReference(speed, ControlType.kVelocity, 0, Constants.Shooter.SHOOTER_FF);
        bottomController.setReference(speed, ControlType.kVelocity, 0, Constants.Shooter.SHOOTER_FF);
    }

    public boolean minimalError() {
        return accuracy > Constants.Shooter.SHOOTER_ACCURACY_REQUIREMENT && (getBottomShooterSpeed() + getTopShooterSpeed()) / 2 > 4500;
    }

    public void setShooterSpeed(double speed) {
        topShooter.set(speed);
        bottomShooter.set(speed);
    }

    public Command sysIdQuasistatic(SysIdRoutine.Direction direction) {
        return sysIdRoutine.quasistatic(direction);
    }

    public Command sysIdDynamic(SysIdRoutine.Direction direction) {
        return sysIdRoutine.dynamic(direction);
    }
}