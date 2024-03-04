package frc.robot.subsystems;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.networktables.DoubleArraySubscriber;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class Limelight extends SubsystemBase {
    private static NetworkTable table = NetworkTableInstance.getDefault().getTable("limelight");
    private final static DoubleArraySubscriber tagPoseTopic = table.getDoubleArrayTopic("targetpose_robotspace").subscribe(new double[6]);
    private static double[] tagPose = new double[6];
    private static int updates;

    //TODO make sure the coordinates here are correct
    public static Pose2d getLimeLightPose(){
        return new Pose2d(getRZ(), getRX(), new Rotation2d(getYaw()));
    }

    @Override
    public void periodic() {
        refreshValues();
    }

    public static boolean tagExists() {
        refreshValues();
        return !(getTable().getEntry("tv").getDouble(0) == 0);
    }

    public static int getUpdates() {
        return updates;
    }

    // X+ is to the right when looking at the tag
    public static double getRX() {
        refreshValues();
        return tagPose[0];
    }

    // Y+ is upwards
    public static double getRY() {
        refreshValues();
        return tagPose[1];
    }

    // Z+ is perpendicular to the plane of the limelight (Z+ is towards tag on data
    // side, Z- is on other side of robot)
    public static double getRZ() {
        refreshValues();
        return tagPose[2];
    }

    public static double getPitch() {
        refreshValues();
        return tagPose[3];
    }

    public static double getYaw() {
        refreshValues();
        return tagPose[4];
    }

    public static double getRoll() {
        refreshValues();
        return tagPose[5];
    }

    // returns lateral angle of tag from center of limelight in degrees
    public static double getLateralOffset() {
        refreshValues();
        return (new Rotation2d(tagPose[2], tagPose[0]).getDegrees() + Constants.Limelight.YAW_OFFSET / tagPose[2]);
    }

    public static NetworkTable getTable() {
        refreshValues();
        return table;
    }

    public static void refreshValues() {
        table = NetworkTableInstance.getDefault().getTable("limelight");
        tagPose = tagPoseTopic.get(new double[6]);
        updates++;
    }
}