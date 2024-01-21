package frc.robot.subsystems;
import edu.wpi.first.networktables.DoubleArraySubscriber;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class Limelight extends SubsystemBase {

    NetworkTable table = NetworkTableInstance.getDefault().getTable("limelight");

    DoubleArraySubscriber ySub;
    public double botPose[];
    public double botPoseX;
    public double botPoseZ;
    public double tagX;
    public double tagY;
    public double tagPresent;
    public int updates;

    public Limelight() {
        ySub = table.getDoubleArrayTopic("targetpose_robotspace").subscribe(new double[6]);
        botPose = new double[6];
        botPoseX = 0.0;
        botPoseZ = 0.0;
    }

    public void refreshValues(){
        table = NetworkTableInstance.getDefault().getTable("limelight");
        botPose = ySub.get(new double[6]);
        tagPresent = table.getEntry("tv").getDouble(0);
    }

    public double getRX(){
        refreshValues();
        return botPose[0];
    }

    public double getRY(){
        refreshValues();
        return botPose[1];
    }

    public double getRZ(){
        refreshValues();
        return botPose[2];
    }

    public double getPitch(){
        refreshValues();
        return botPose[3];
    }

    public double getYaw(){
        refreshValues();
        return botPose[4];
    }

    public double getRoll(){
        refreshValues();
        return botPose[5];
    }
}