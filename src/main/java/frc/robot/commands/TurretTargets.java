package frc.robot.commands;

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
}
