package frc.robot.subsystems;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.lib.util.LimelightHelpers;
import frc.robot.Constants;

public class Limelight extends SubsystemBase {
    private static double[] tagPose = LimelightHelpers.getTargetPose_RobotSpace("limelight");
    public static boolean overrideTargetNow = false; // only for auto

    @Override
    public void periodic() {
        refreshValues();
    }

    public static boolean tagExists() {
        return !LimelightHelpers.getTV("limelight");
    }

    // X+ is to the right when looking at the tag
    public static double getTagRX() {
        return tagPose[0];
    }

    // Y+ is upwards
    public static double getTagRY() {
        return tagPose[1];
    }

    // Z+ is perpendicular to the plane of the limelight (Z+ is towards tag on data
    // side, Z- is on other side of robot)
    public static double getTagRZ() {
        return tagPose[2];
    }

    public static double getTagPitch() {
        return tagPose[3];
    }

    public static double getTagYaw() {
        return tagPose[4];
    }

    public static double getTagRoll() {
        return tagPose[5];
    }

    // returns lateral angle of tag from center of limelight in degrees
    public static double getTagLateralAngle() {
        if (LimelightHelpers.getFiducialID("limelight") == 7.0) {
            return (new Rotation2d(tagPose[2], tagPose[0]).getDegrees() + Constants.Limelight.YAW_OFFSET / tagPose[2]);
        }
        return 0.0;
    }

    public static void refreshValues() {
        tagPose = LimelightHelpers.getTargetPose_RobotSpace("limelight");
    }
}