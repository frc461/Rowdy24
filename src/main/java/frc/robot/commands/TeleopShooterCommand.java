package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants;
import frc.robot.subsystems.Shooter;
import frc.robot.subsystems.Swerve;

public class TeleopShooterCommand extends Command {
    private final Shooter shooter;
    private final Swerve swerve;

    public TeleopShooterCommand(Shooter shooter, Swerve swerve) {
        this.shooter = shooter;
        this.swerve = swerve;
        addRequirements(shooter);
    }

    @Override
    public void initialize() {
        shooter.setShooterSpeed(Constants.Shooter.IDLE_SHOOTER_POWER);
    }

    @Override
    public void execute() {
        if (swerve.getVectorToSpeakerTarget().getNorm() < Constants.Shooter.SHOOTER_TRIGGER_DISTANCE) {
            shooter.shoot(Constants.Shooter.BASE_SHOOTER_SPEED +
                    swerve.getVectorToSpeakerTarget().getNorm() * Constants.Shooter.DISTANCE_MULTIPLIER);
        } else {
            shooter.setShooterSpeed(Constants.Shooter.IDLE_SHOOTER_POWER);
        }
    }
}