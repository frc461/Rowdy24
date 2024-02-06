package frc.lib.util;

import edu.wpi.first.math.geometry.Rotation2d;

public class SwerveModuleConstants {
    public final int driveMotorID;
    public final int angleMotorID;
    public final int cancoderID;
    public final Rotation2d angleOffset;

    /**
     * Swerve Module Constants to be used when creating swerve modules.
     * @param driveMotorID ID of the drive motor
     * @param angleMotorID ID of the angle motor
     * @param canCoderID ID of the CANcoder
     * @param angleOffset Angle offset of the module
     */
    public SwerveModuleConstants(int driveMotorID, int angleMotorID, int canCoderID, Rotation2d angleOffset, boolean antiXMode) {
        this.driveMotorID = driveMotorID;
        this.angleMotorID = angleMotorID;
        this.cancoderID = canCoderID;
        this.angleOffset = antiXMode ? Rotation2d.fromDegrees(angleOffset.getDegrees() + 90.0) : angleOffset;
    }
}
