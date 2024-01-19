package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.Swerve;
import frc.robot.subsystems.Limelight;

public class LimelightFollow extends Command {
    private Limelight s_limelight;
    private Swerve swerve;
    
    public LimelightFollow(Limelight limelight, Swerve swerve) {
        s_limelight = limelight;
        this.swerve = swerve;
        addRequirements(s_limelight, swerve);
    }

    @Override
    public void execute() {
        swerve.rotateDegrees(s_limelight.getYaw());
    }
    
}
