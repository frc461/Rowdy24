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

    private static double[] tagPose = new double[6];

    @Override
    public void periodic() {
        refreshValues();
    }

    public static boolean tagExists() {
        return !(table.getEntry("tv").getDouble(0) == 0);
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

    public static NetworkTable getTable() {
        refreshValues();
        return table;
    }

    public static void refreshValues() {
        table = NetworkTableInstance.getDefault().getTable("limelight");
        tagPose = tagPoseTopic.get(new double[6]);
    }
}