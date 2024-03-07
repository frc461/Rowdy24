package frc.robot.subsystems;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.networktables.DoubleArraySubscriber;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class Limelight extends SubsystemBase {
    private static NetworkTable table = NetworkTableInstance.getDefault().getTable("limelight");
    private final static DoubleArraySubscriber tagPoseTopic = table.getDoubleArrayTopic("targetpose_robotspace").subscribe(new double[6]);
    private final static DoubleArraySubscriber fusedPoseTopic = table.getDoubleArrayTopic("botpose_wpiblue").subscribe(new double[11]);

    private static double[] tagPose = new double[6];
    private static double[] fusedPose = new double[11]; // TODO: figure out what this network table consists of

    public static Pose2d getBluePoseRobotSpace() {
        return new Pose2d(getFusedRX(), getFusedRY(), new Rotation2d(getFusedYaw()));
    }

    @Override
    public void periodic() {
        refreshValues();
    }

    public static boolean tagExists() {
        return !(table.getEntry("tv").getDouble(0) == 0);
    }
    public static double[] getFusedPoseTable() {
        return fusedPose;
    }

    public static double getFusedRX() {
        return fusedPose[0];
    }
   
    public static double getFusedRY() {
        return fusedPose[1];
    }
    
    public static double getFusedRZ() {
        return fusedPose[2];
    }

    public static double getFusedPitch() {
        return fusedPose[3];
    }
    
    public static double getFusedRoll() {
        return fusedPose[4];
    }

    public static double getFusedYaw() {
        return fusedPose[5];
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
        return (new Rotation2d(tagPose[2], tagPose[0]).getDegrees() + Constants.Limelight.YAW_OFFSET / tagPose[2]);
    }

    public static void refreshValues() {
        table = NetworkTableInstance.getDefault().getTable("limelight");
        tagPose = tagPoseTopic.get(new double[6]);
        fusedPose = fusedPoseTopic.get(new double[6]);
    }
}