package frc.robot.commands;
import java.util.function.BooleanSupplier;
import java.util.function.DoubleSupplier;
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.Swerve;
import frc.robot.constants.variants.PracticeConstants;
import frc.robot.constants.Configuration;
import frc.robot.constants.RobotConstants;
import frc.robot.constants.RobotIdentity;
import frc.robot.subsystems.Limelight;

public class LimelightTurretCommand extends Command {
    private final Limelight limelight;
    private final Swerve swerve;
    private final DoubleSupplier translationSup;
    private final DoubleSupplier strafeSup;
    private final BooleanSupplier robotCentricSup;
    private final RobotConstants robot = RobotConstants.getRobotConstants(RobotIdentity.getIdentity());
    private final Configuration configuration = robot.getConfiguration();

    public LimelightTurretCommand(
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
        addRequirements(this.limelight, this.swerve);
    }
    
    @Override
    public void execute() {
        /* Apply Deadband */
        double translationVal = MathUtil.applyDeadband(translationSup.getAsDouble(), PracticeConstants.STICK_DEADBAND);
        double strafeVal = MathUtil.applyDeadband(strafeSup.getAsDouble(), PracticeConstants.STICK_DEADBAND);

        /* Calculate Rotation Magnitude */
        if(limelight.tagExists()) {
            try (
                    PIDController rotController = new PIDController(
                            configuration.limelight_p,
                            configuration.limelight_i,
                            configuration.limelight_d
                    )
            ) {
                rotController.enableContinuousInput(PracticeConstants.MINIMUM_ANGLE, PracticeConstants.MAXIMUM_ANGLE);

                double rotate = rotController.calculate(
                        swerve.getYaw(),
                        swerve.getYaw() + limelight.getLateralOffset()
                );

                /* Drive */
                swerve.drive(
                    new Translation2d(translationVal, strafeVal).times(configuration.max_speed),
                    -rotate,
                    !robotCentricSup.getAsBoolean(),
                    true
                );
            }
        }
    }
}
