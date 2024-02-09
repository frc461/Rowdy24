package frc.robot.commands;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj2.command.Command;

import frc.robot.subsystems.Limelight;
import frc.robot.subsystems.Swerve;
import frc.robot.Constants;
import frc.robot.subsystems.Angler;

public class AutoAllign extends Command {

    private final Angler angler;
    private final Limelight limelight;
    private final Swerve swerve;

    public AutoAllign(Swerve swerve, Angler angler, Limelight limelight) {
        this.angler = angler;
        this.limelight = limelight;
        this.swerve = swerve;
        addRequirements(swerve, angler, limelight);
    }

    @Override
    public void execute() {
    
        //TODO actually make this work
       angler.setAngle(0);
       
        try (
                PIDController rotController = new PIDController(
                        Constants.Limelight.LIMELIGHT_P,
                        Constants.Limelight.LIMELIGHT_I,
                        Constants.Limelight.LIMELIGHT_D
                )
        ) {
            rotController.enableContinuousInput(Constants.MINIMUM_ANGLE, Constants.MAXIMUM_ANGLE);

            // TODO: verify angle
            double rotate = rotController.calculate(
                    swerve.getYaw(),
                    swerve.getYaw() + Math.atan(
                            limelight.getRX() / limelight.getRZ()
                    ) * 180 / Math.PI
            );

            /* Drive */
            swerve.drive(
                new Translation2d(0, 0).times(Constants.Swerve.MAX_SPEED),
                -rotate,
                true,
                true
            );
        }











    }

}
