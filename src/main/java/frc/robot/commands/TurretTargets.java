package frc.robot.commands;

import edu.wpi.first.math.geometry.Rotation2d;
import frc.robot.Constants;
import frc.robot.subsystems.Swerve;

public enum TurretTargets {
    SPEAKER,
    SHUTTLE;

    public static double getAngleToTarget(TurretTargets targetType, Swerve swerve) {
        return switch (targetType) {
            case SPEAKER -> swerve.getAngleToSpeakerTarget();
            case SHUTTLE -> swerve.getAngleToShuttleTarget();
        };
    }

    public static double calculateAngularRate(double angleToTarget, double currentAngle) {
        double angleDifference = Rotation2d.fromDegrees(angleToTarget).minus(Rotation2d.fromDegrees(currentAngle)).getDegrees();
        return Constants.Limelight.ANGLE_DIFFERENCE_TO_RATE.apply(angleDifference);
    }
}
