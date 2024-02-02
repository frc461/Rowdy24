package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants;
import frc.robot.subsystems.Shooter;

public class AutoShooter extends Command {

    private final Shooter shooter;

    public AutoShooter(Shooter shooter) {
        this.shooter = shooter;
        addRequirements(shooter);
    }

    @Override
    public void execute() {
        shooter.shoot(Constants.Shooter.BASE_SHOOTER_SPEED, false);
    }

}