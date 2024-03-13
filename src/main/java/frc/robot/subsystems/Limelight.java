package frc.robot.subsystems;

import org.photonvision.PhotonCamera;
import org.photonvision.proto.Photon;
import org.photonvision.targeting.PhotonPipelineResult;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.networktables.DoubleArraySubscriber;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class Limelight extends SubsystemBase {
    private static PhotonCamera camera = new PhotonCamera("photonvision");
    private static PhotonPipelineResult result = camera.getLatestResult();
    // private static double[] tagPose = new double[6];
    private static int updates;

    @Override
    public void periodic() {
        refreshValues();
    }

    public static boolean tagExists() {
        refreshValues();
        return result.hasTargets();
    }

    public static int getUpdates() {
        return updates;
    }

    // X+ is to the right when looking at the tag
    // The tag is gotten in camera space (relative to camera position)
    public static double getRX() {
        refreshValues();
        return result.getBestTarget().getBestCameraToTarget().getX();
    }

    // Y+ is upwards
    public static double getRY() {
        refreshValues();
        return result.getBestTarget().getBestCameraToTarget().getY();
    }

    // Z+ is perpendicular to the plane of the limelight (Z+ is towards tag on data
    // side, Z- is on other side of robot)
    public static double getRZ() {
        refreshValues();
        return result.getBestTarget().getBestCameraToTarget().getZ();
    }

    public static double getPitch() {
        refreshValues();
        return result.getBestTarget().getPitch();
    }

    public static double getYaw() {
        refreshValues();
        return result.getBestTarget().getYaw();
    }


    //This is an unused functin, and photonVision does not return roll


    // public static double getRoll() {
    //     refreshValues();
    //     return result.getBestTarget().getRoll();
    // }


    // returns lateral angle of tag from center of limelight in degrees
    public static double getLateralOffset() {
        refreshValues();
        return (new Rotation2d(
            result.getBestTarget().getBestCameraToTarget().getZ(),
            result.getBestTarget().getBestCameraToTarget().getX()
            )
            .getDegrees() + Constants.Limelight.YAW_OFFSET / 
            result.getBestTarget().getBestCameraToTarget().getZ());
    }

    public static void refreshValues() {
        result = camera.getLatestResult();
        updates++;
    }
}