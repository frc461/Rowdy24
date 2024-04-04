package frc.robot.subsystems;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.lib.util.LimelightHelpers;
import frc.lib.util.TagLocation;
import frc.robot.Constants;

public class Limelight extends SubsystemBase {
    private static double[] tagPose = LimelightHelpers.getTargetPose_RobotSpace("limelight");
    public static boolean overrideTargetNow = false; // only for auto

    @Override
    public void periodic() {
        refreshValues();
    }

    public static boolean tagExists() {
        return LimelightHelpers.getTV("limelight");
    }

    public static int getNumTag() {
        return (int) LimelightHelpers.extractBotPoseEntry(LimelightHelpers.getBotPose_wpiBlue("limelight"), 7);
    }

    public static Pose2d getNearestTagPose() {
        return TagLocation.getTagLocation(LimelightHelpers.getFiducialID("limelight"));
    }

    // X+ is to the right when looking at the tag
    public static double getTagRX() {
        return tagPose.length == 0 ? 0.0 : tagPose[0];
    }

    // Y+ is upwards
    public static double getTagRY() {
        return tagPose.length == 0 ? 0.0 : tagPose[1];
    }

    // Z+ is perpendicular to the plane of the limelight (Z+ is towards tag on data
    // side, Z- is on other side of robot)
    public static double getTagRZ() {
        return tagPose.length == 0 ? 0.0 : tagPose[2];
    }

    public static double getTagPitch() {
        return tagPose.length == 0 ? 0.0 : tagPose[3];
    }

    public static double getTagYaw() {
        return tagPose.length == 0 ? 0.0 : tagPose[4];
    }

    public static double getTagRoll() {
        return tagPose.length == 0 ? 0.0 : tagPose[5];
    }

    public static boolean isRedAlliance() {
        return DriverStation.getAlliance().filter(value -> value == DriverStation.Alliance.Red).isPresent();
    }

    public static Pose2d getSpeakerTagPose() {
        return TagLocation.getTagLocation(
                isRedAlliance() ? TagLocation.ID_4 : TagLocation.ID_7
        );
    }

    // returns lateral angle of tag from center of limelight in degrees
    public static double getTagLateralAngle() {
        return (new Rotation2d(tagPose[2], tagPose[0]).getDegrees() + Constants.Limelight.YAW_OFFSET / tagPose[2]);
    }

    public static void refreshValues() {
        tagPose = LimelightHelpers.getTargetPose_RobotSpace("limelight");
    }

    public static void configureRobotPose() {
        // 0.0223774 m forward, 0.5916676 m up, 23.6838871 deg pitch
        LimelightHelpers.setCameraPose_RobotSpace(
                "limelight",
                Constants.Limelight.CAMERA_FORWARD,
                0,
                Constants.Limelight.CAMERA_UP,
                0,
                Constants.Limelight.CAMERA_PITCH,
                0
        );
    }
}