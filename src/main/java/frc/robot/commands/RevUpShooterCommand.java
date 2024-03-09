package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants;
import frc.robot.subsystems.Limelight;
import frc.robot.subsystems.Shooter;

public class RevUpShooterCommand extends Command {
    private final Shooter shooter;
    
    public RevUpShooterCommand(Shooter shooter) {
        this.shooter = shooter;
        addRequirements(this.shooter);
    }

    @Override
    public void initialize() {
        shooter.shoot(Constants.Shooter.BASE_SHOOTER_SPEED + Limelight.getTagRZ() * Constants.Shooter.DISTANCE_MULTIPLIER);
    }

    @Override
    public void execute() {
        shooter.shoot(Constants.Shooter.BASE_SHOOTER_SPEED + Limelight.getTagRZ() * Constants.Shooter.DISTANCE_MULTIPLIER);
    }

    @Override
    public void end(boolean isFinished) {
        shooter.setShooterSpeed(0);
    }
}
