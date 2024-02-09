package frc.robot.commands;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.wpilibj2.command.Command;

import frc.robot.subsystems.Limelight;
import frc.robot.subsystems.Swerve;
import frc.robot.Constants;
import frc.robot.subsystems.Angler;

public class AutoAlign extends Command {

    private final Angler angler;
    private final Limelight limelight;
    private final Swerve swerve;

    public AutoAlign(Swerve swerve, Angler angler, Limelight limelight) {
        this.angler = angler;
        this.limelight = limelight;
        this.swerve = swerve;
        addRequirements(swerve, angler, limelight);
    }

    @Override
    public void execute() {
        //TODO: VERIFY METHODS

        /* Calculate angler pitch-wise trajectory */
        angler.setAlignedAngle(limelight.getRX(), limelight.getRZ());

        /* Use PID to turret-aim to speaker while moving with current swerve module states */
        try (
                PIDController rotController = new PIDController(
                        Constants.Limelight.LIMELIGHT_P,
                        Constants.Limelight.LIMELIGHT_I,
                        Constants.Limelight.LIMELIGHT_D
                )
        ) {
            rotController.enableContinuousInput(Constants.MINIMUM_ANGLE, Constants.MAXIMUM_ANGLE);

            ChassisSpeeds speeds = Constants.Swerve.SWERVE_KINEMATICS.toChassisSpeeds(swerve.getModuleStates());

            double rotate = rotController.calculate(
                    swerve.getYaw(),
                    swerve.getYaw() + limelight.getLateralOffset()
            );

            /* Drive */
            swerve.drive(
                new Translation2d(speeds.vxMetersPerSecond, speeds.vyMetersPerSecond),
                rotate,
                true,
                true
            );
        }
    }
}
