package frc.robot.subsystems;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.lib.util.LimelightHelpers;
import frc.lib.util.TagLocation;
import frc.robot.Constants;

public class Limelight extends SubsystemBase {
    public static boolean overrideTargetNow = false; // only for auto

    public static double[] getBotPoseBlueTableMegaTag2(String limelightName) {
        return LimelightHelpers.getLimelightNTDoubleArray(limelightName, "botpose_orb_wpiblue");
    }

    public static Pose2d getBotPoseBlueMegaTag2(String limelightName) {
        return LimelightHelpers.toPose2D(getBotPoseBlueTableMegaTag2(limelightName));
    }

    public static double[] getTargetPoseRobotSpaceTable(String limelightName) {
        return LimelightHelpers.getLimelightNTDoubleArray(limelightName, "targetpose_robotspace");
    }

    public static Pose2d getTargetPoseRobotSpace(String limelightName) {
        double[] inData = getTargetPoseRobotSpaceTable(limelightName);
        
        Translation2d tran2d = new Translation2d(inData[0], inData[2]);
        Rotation2d r2d = new Rotation2d(Units.degreesToRadians(inData[5]));
        return new Pose2d(tran2d, r2d);
    }

    public static boolean tagExists() {
        return LimelightHelpers.getTV("limelight");
    }

    public static int getNumTag() {
        return (int) LimelightHelpers.extractBotPoseEntry(LimelightHelpers.getBotPose_wpiBlue("limelight"), 7);
    }

    public static double getNearestTagDist() {
        return getTargetPoseRobotSpace("limelight").getTranslation().getNorm();
    }

    public static boolean isRedAlliance() {
        return DriverStation.getAlliance().filter(value -> value == DriverStation.Alliance.Red).isPresent();
    }

    public static Pose2d getSpeakerTagPose() {
        return TagLocation.getTagLocation(
                isRedAlliance() ? TagLocation.ID_4 : TagLocation.ID_7
        );
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