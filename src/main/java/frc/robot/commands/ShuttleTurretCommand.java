package frc.robot.commands;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants;
import frc.robot.subsystems.Swerve;

import java.util.function.BooleanSupplier;
import java.util.function.DoubleSupplier;

public class ShuttleTurretCommand extends Command {
    private final Swerve swerve;
    private final DoubleSupplier translationSup;
    private final DoubleSupplier strafeSup;
    private final BooleanSupplier robotCentricSup;

    public ShuttleTurretCommand(
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
        double translationVal = MathUtil.applyDeadband(translationSup.getAsDouble(), Constants.STICK_DEADBAND);
        double strafeVal = MathUtil.applyDeadband(strafeSup.getAsDouble(), Constants.STICK_DEADBAND);

        swerve.driveShuttleTurret(
                new Translation2d(
                        translationVal * Constants.Swerve.MAX_SPEED,
                        strafeVal * Constants.Swerve.MAX_SPEED
                ),
                !robotCentricSup.getAsBoolean()
        );
    }

    @Override
    public void end(boolean isFinished) {
        swerve.drive(new Translation2d(0, 0), 0.0, true, true);
    }
}