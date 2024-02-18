package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants;
import frc.robot.subsystems.Limelight;
import frc.robot.subsystems.Shooter;

public class RevUpShooterCommand extends Command {
    private final Shooter shooter;
    private final Limelight limelight;
    private final boolean idleMode;
    public RevUpShooterCommand(Shooter shooter, Limelight limelight, boolean idleMode) {
        this.shooter = shooter;
        this.limelight = limelight;
        this.idleMode = idleMode;
        addRequirements(this.shooter);
    }

    @Override
    public void initialize() {
        shooter.shoot(Constants.Shooter.BASE_SHOOTER_SPEED + limelight.getRZ() * Constants.Shooter.DISTANCE_MULTIPLIER);
    }

    @Override
    public void execute() {
        shooter.shoot(Constants.Shooter.BASE_SHOOTER_SPEED + limelight.getRZ() * Constants.Shooter.DISTANCE_MULTIPLIER);
    }

    @Override
    public void end(boolean isFinished) {
        shooter.setShooterIdle(idleMode);
    }
}
