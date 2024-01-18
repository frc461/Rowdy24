package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.Swerve;
import frc.robot.subsystems.Limelight;

public class LimelightFollow extends Command {
    private Limelight s_limelight;
    private Swerve s_Swerve;
    
    public LimelightFollow(Limelight lime, Swerve swerve) {
        s_limelight = lime;
        s_Swerve = swerve;
        addRequirements(s_limelight, s_Swerve);
    }

    @Override
    public void execute() {

        if(s_limelight.isTagPresent() != 0) {
            s_Swerve.rotateToDegree(s_limelight.getYaw());
        }
    }
    
}
