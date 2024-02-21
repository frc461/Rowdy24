package frc.robot.commands;

import java.util.function.BooleanSupplier;
import java.util.function.DoubleSupplier;
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.constants.variants.PracticeConstants;
import frc.robot.constants.Configuration;
import frc.robot.constants.RobotConstants;
import frc.robot.constants.RobotIdentity;
import frc.robot.subsystems.Swerve;

public class TeleopSwerveCommand extends Command {
    private final Swerve swerve;
    private final DoubleSupplier translationSup;
    private final DoubleSupplier strafeSup;
    private final DoubleSupplier rotationSup;
    private final BooleanSupplier robotCentricSup;
    private final RobotConstants robot = RobotConstants.getRobotConstants(RobotIdentity.getIdentity());
    private final Configuration configuration = robot.getConfiguration();

    public TeleopSwerveCommand(
            Swerve swerve,
            DoubleSupplier translationSup,
            DoubleSupplier strafeSup,
            DoubleSupplier rotationSup,
            BooleanSupplier robotCentricSup
    ) {
        this.swerve = swerve;
        this.translationSup = translationSup;
        this.strafeSup = strafeSup;
        this.rotationSup = rotationSup;
        this.robotCentricSup = robotCentricSup;
        addRequirements(this.swerve);
    }

    @Override
    public void execute() {
        /* Get Values, Deadband */
        double translationVal = MathUtil.applyDeadband(translationSup.getAsDouble(), PracticeConstants.STICK_DEADBAND);
        double strafeVal = MathUtil.applyDeadband(strafeSup.getAsDouble(), PracticeConstants.STICK_DEADBAND);
        double rotationVal = MathUtil.applyDeadband(rotationSup.getAsDouble(), PracticeConstants.STICK_DEADBAND);

        /* Drive */
        swerve.drive(
                new Translation2d(translationVal, strafeVal).times(configuration.max_speed),
                rotationVal * configuration.max_angular_velocity,
                !robotCentricSup.getAsBoolean(),
                true
        );
    }
}