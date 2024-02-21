package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.constants.Configuration;
import frc.robot.constants.RobotConstants;
import frc.robot.constants.RobotIdentity;
import frc.robot.subsystems.Limelight;
import frc.robot.subsystems.Shooter;

public class RevUpShooterCommand extends Command {
    private final Shooter shooter;
    private final Limelight limelight;
    private final boolean idleMode;
    private final RobotConstants robot = RobotConstants.getRobotConstants(RobotIdentity.getIdentity());
    private final Configuration configuration = robot.getConfiguration();

    public RevUpShooterCommand(Shooter shooter, Limelight limelight, boolean idleMode) {
        this.shooter = shooter;
        this.limelight = limelight;
        this.idleMode = idleMode;
        addRequirements(this.shooter);
    }

    @Override
    public void initialize() {
        shooter.shoot(configuration.base_shooter_speed + limelight.getRZ() * configuration.distance_multiplier);
    }

    @Override
    public void execute() {
        shooter.shoot(configuration.base_shooter_speed + limelight.getRZ() * configuration.distance_multiplier);
    }

    @Override
    public void end(boolean isFinished) {
        shooter.setShooterIdle(idleMode);
    }
}
