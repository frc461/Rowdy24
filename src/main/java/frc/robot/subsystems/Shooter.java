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
    private final SparkPIDController bottomPIDController, topPIDController;
    private final RelativeEncoder bottomEncoder, topEncoder;
    private final MutableMeasure<Voltage> appliedVoltage = mutable(Volts.of(0));
    private final MutableMeasure<Angle> angle = mutable(Rotations.of(0));
    private final MutableMeasure<Velocity<Angle>> velocity = mutable(RPM.of(0));
    private final SysIdRoutine sysIdRoutine;
    private double target, error;

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

        topPIDController = topShooter.getPIDController();
        bottomPIDController = bottomShooter.getPIDController();

        topPIDController.setP(Constants.Shooter.SHOOTER_P);
        topPIDController.setI(Constants.Shooter.SHOOTER_I);
        topPIDController.setD(Constants.Shooter.SHOOTER_D);
        topPIDController.setFF(Constants.Shooter.SHOOTER_FF);

        bottomPIDController.setP(Constants.Shooter.SHOOTER_P);
        bottomPIDController.setI(Constants.Shooter.SHOOTER_I);
        bottomPIDController.setD(Constants.Shooter.SHOOTER_D);
        bottomPIDController.setFF(Constants.Shooter.SHOOTER_FF);
        
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
        error = Math.abs(target - (getBottomShooterSpeed() + getTopShooterSpeed()) / 2);
    }

    @Override
    public void periodic() {
        error = Math.abs(target - (getBottomShooterSpeed() + getTopShooterSpeed()) / 2);
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
        topPIDController.setReference(speed, ControlType.kVelocity, 0, Constants.Shooter.SHOOTER_FF);
        bottomPIDController.setReference(speed, ControlType.kVelocity, 0, Constants.Shooter.SHOOTER_FF);
    }

    public boolean minimalError() {
        return error < Constants.Shooter.SHOOTER_ERROR_TOLERANCE && (getBottomShooterSpeed() + getTopShooterSpeed()) / 2 > 5000;
    }

    public void setShooterIdle(boolean idleMode) {
        bottomShooter.set(idleMode ? Constants.Shooter.IDLE_SHOOTER_SPEED : 0);
        topShooter.set(idleMode ? Constants.Shooter.IDLE_SHOOTER_SPEED : 0);
    }

    public void overrideShooterSpeed(double speed) {
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