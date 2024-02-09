package frc.robot.commands;
import java.util.function.BooleanSupplier;
import java.util.function.DoubleSupplier;
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.Swerve;
import frc.robot.Constants;
import frc.robot.subsystems.Limelight;

public class TeleopLimelightTurret extends Command {
    private final Limelight limelight;
    private final Swerve swerve;
    private final DoubleSupplier translationSup;
    private final DoubleSupplier strafeSup;
    private final BooleanSupplier robotCentricSup;

    public TeleopLimelightTurret(
            Limelight limelight,
            Swerve swerve,
            DoubleSupplier translationSup,
            DoubleSupplier strafeSup,
            BooleanSupplier robotCentricSup
    ) {
        this.limelight = limelight;
        this.swerve = swerve;
        this.translationSup = translationSup;
        this.strafeSup = strafeSup;
        this.robotCentricSup = robotCentricSup;
        addRequirements(this.limelight, swerve);
    }
    
    @Override
    public void execute() {
        /* Apply Deadband */
        double translationVal = MathUtil.applyDeadband(translationSup.getAsDouble(), Constants.STICK_DEADBAND);
        double strafeVal = MathUtil.applyDeadband(strafeSup.getAsDouble(), Constants.STICK_DEADBAND);

        /* Calculate Rotation Magnitude */
        try (
                PIDController rotController = new PIDController(
                        Constants.Limelight.LIMELIGHT_P,
                        Constants.Limelight.LIMELIGHT_I,
                        Constants.Limelight.LIMELIGHT_D
                )
        ) {
            rotController.enableContinuousInput(Constants.MINIMUM_ANGLE, Constants.MAXIMUM_ANGLE);

            double rotate = rotController.calculate(
                    swerve.getYaw(),
                    swerve.getYaw() + limelight.getLateralOffset()
            );

            /* Drive */
            swerve.drive(
                new Translation2d(translationVal, strafeVal).times(Constants.Swerve.MAX_SPEED),
                rotate,
                !robotCentricSup.getAsBoolean(),
                true
            );
        }
    }
}
