package frc.robot.commands;
import java.util.function.BooleanSupplier;
import java.util.function.DoubleSupplier;
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.Swerve;
import frc.robot.Constants;

public class TurretCommand extends Command {
    private final Swerve swerve;
    private final DoubleSupplier translationSup;
    private final DoubleSupplier strafeSup;
    private final BooleanSupplier robotCentricSup;
    private final Swerve.TurretTargets targetType;

    public TurretCommand(
            Swerve swerve,
            DoubleSupplier translationSup,
            DoubleSupplier strafeSup,
            BooleanSupplier robotCentricSup,
            Swerve.TurretTargets targetType
    ) {
        this.swerve = swerve;
        this.translationSup = translationSup;
        this.strafeSup = strafeSup;
        this.robotCentricSup = robotCentricSup;
        this.targetType = targetType;
    }
    
    @Override
    public void execute() {
        /* Apply Deadband */
        double translationVal = MathUtil.applyDeadband(translationSup.getAsDouble(), Constants.STICK_DEADBAND);
        double strafeVal = MathUtil.applyDeadband(strafeSup.getAsDouble(), Constants.STICK_DEADBAND);

        swerve.driveTurret(
                new Translation2d(
                        translationVal * Constants.Swerve.MAX_SPEED,
                        strafeVal * Constants.Swerve.MAX_SPEED
                ),
                !robotCentricSup.getAsBoolean(),
                targetType
        );
    }

    @Override
    public void end(boolean isFinished) {
        swerve.drive(new Translation2d(0, 0), 0.0, true, true);
    }
}