package frc.robot.commands;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.Limelight;
import frc.robot.subsystems.Swerve;

public class FusedOdometryCommand extends Command{
    Limelight limelight;
    Swerve swerve;

    public FusedOdometryCommand(Limelight limelight, Swerve swerve){
        this.limelight = limelight;
        this.swerve = swerve;
        addRequirements(swerve);
        addRequirements(limelight);
    }

    @Override
    public void execute() {
        swerve.updateFusedVision(limelight.getLimeLightPose());
    }
}
