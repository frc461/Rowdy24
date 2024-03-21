package frc.robot.commands;
import java.util.function.BooleanSupplier;
import java.util.function.DoubleSupplier;
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.Swerve;
import frc.robot.Constants;

public class LimelightTurretCommand extends Command {
    private final Swerve swerve;
    private final DoubleSupplier translationSup;
    private final DoubleSupplier strafeSup;
    private final BooleanSupplier robotCentricSup;

    public LimelightTurretCommand(
            Swerve swerve,
            DoubleSupplier translationSup,
            DoubleSupplier strafeSup,
            BooleanSupplier robotCentricSup
    ) {
        this.swerve = swerve;
        this.translationSup = translationSup;
        this.strafeSup = strafeSup;
        this.robotCentricSup = robotCentricSup;
    }
    
    @Override
    public void execute() {
        /* Apply Deadband */

        swerve.driveTurret(
                new Translation2d(translationSup.getAsDouble(), strafeSup.getAsDouble()),
                !robotCentricSup.getAsBoolean()
        );
    }

    @Override
    public void end(boolean isFinished) {
        swerve.drive(new Translation2d(0, 0), 0.0, true, true);
    }
}