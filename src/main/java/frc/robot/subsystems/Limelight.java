package frc.robot.subsystems;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.networktables.DoubleArraySubscriber;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.constants.Constants;
import frc.robot.constants.RobotConstants;
import frc.robot.constants.RobotIdentity;

public class Limelight extends SubsystemBase {
    private final DoubleArraySubscriber tagPoseTopic;
    private NetworkTable table;
    private double[] tagPose;
    private int updates;    private final RobotConstants robot = RobotConstants.getRobotConstants(RobotIdentity.getIdentity());
    private final Constants constants = robot.getConfiguration();

    public Limelight() {
        table = NetworkTableInstance.getDefault().getTable("limelight");
        tagPoseTopic = table.getDoubleArrayTopic("targetpose_robotspace").subscribe(new double[6]);
        tagPose = new double[6];
    }

    @Override
    public void periodic() {
        refreshValues();
    }

    public boolean tagExists() {
        refreshValues();
        return !(getTable().getEntry("tv").getDouble(0) == 0);
    }

    public int getUpdates() {
        return updates;
    }

    // X+ is to the right when looking at the tag
    public double getRX() {
        refreshValues();
        return tagPose[0];
    }

    // Y+ is upwards
    public double getRY() {
        refreshValues();
        return tagPose[1];
    }

    // Z+ is perpendicular to the plane of the limelight (Z+ is towards tag on data
    // side, Z- is on other side of robot)
    public double getRZ() {
        refreshValues();
        return tagPose[2];
    }

    public double getPitch() {
        refreshValues();
        return tagPose[3];
    }

    public double getYaw() {
        refreshValues();
        return tagPose[4];
    }

    public double getRoll() {
        refreshValues();
        return tagPose[5];
    }

    // returns lateral angle of tag from center of limelight in degrees
    public double getLateralOffset() {
        refreshValues();
        return (new Rotation2d(tagPose[2], tagPose[0]).getDegrees() + Constants.Limelight.YAW_OFFSET / tagPose[2]);
    }

    public NetworkTable getTable() {
        refreshValues();
        return table;
    }

    public void refreshValues() {
        table = NetworkTableInstance.getDefault().getTable("limelight");
        tagPose = tagPoseTopic.get(new double[6]);
        updates++;
    }
}