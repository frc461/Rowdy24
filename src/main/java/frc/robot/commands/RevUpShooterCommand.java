package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants;
import frc.robot.subsystems.Shooter;
import frc.robot.subsystems.Swerve;

public class RevUpShooterCommand extends Command {
    private final Shooter shooter;
    private final Swerve swerve;
    
    public RevUpShooterCommand(Shooter shooter, Swerve swerve) {
        this.shooter = shooter;
        this.swerve = swerve;
        addRequirements(this.shooter);
    }

    @Override
    public void execute() {
        shooter.shoot(Constants.Shooter.BASE_SHOOTER_SPEED +
                swerve.getVectorToSpeakerTarget().getNorm() * Constants.Shooter.DISTANCE_MULTIPLIER);
    }

    @Override
    public void end(boolean isFinished) {
        shooter.setShooterSpeed(0);
    }
}
