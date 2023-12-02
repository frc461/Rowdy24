package frc.robot.commands;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.Swerve;
import frc.robot.subsystems.Limelight;

public class TeleopLimelight extends Command {
    private Limelight limelight;

    public TeleopLimelight(Limelight limelight) {
        this.limelight = limelight;
        // this.swerve = swerve;
        addRequirements(limelight);
    }
    
    @Override
    public void execute() {
        limelight.refreshValues();
        // swerve.resetOdometry(
        //     new Pose2d(
        //         limelight.botPoseZ,
        //         -limelight.botPoseX,
        //         Rotation2d.fromDegrees(limelight.botPose[4])
        //     )
        // );
    }
}

