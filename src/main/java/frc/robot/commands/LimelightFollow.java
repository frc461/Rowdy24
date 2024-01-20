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

public class LimelightFollow extends Command {
    private Limelight s_limelight;
    private Swerve s_Swerve;    
    private DoubleSupplier translationSup;
    private DoubleSupplier strafeSup;
    private DoubleSupplier rotationSup;
    private BooleanSupplier robotCentricSup;

   

    
    public LimelightFollow(Limelight limelight, Swerve s_Swerve, DoubleSupplier translationSup, DoubleSupplier strafeSup, DoubleSupplier rotationSup, BooleanSupplier robotCentricSup) {
        s_limelight = limelight;
        this.s_Swerve = s_Swerve;
        this.translationSup = translationSup;
        this.strafeSup = strafeSup;
        this.rotationSup = rotationSup;
        this.robotCentricSup = robotCentricSup;
        addRequirements(s_limelight, s_Swerve);
    }

    @Override
    public void execute() {

        s_limelight.refreshValues();

       /* Get Values, Deadband*/
        double translationVal = MathUtil.applyDeadband(translationSup.getAsDouble(), Constants.stickDeadband);
        double strafeVal = MathUtil.applyDeadband(strafeSup.getAsDouble(), Constants.stickDeadband);

        /* Get rotation */
        PIDController rotController = new PIDController(0.3, 0.0008, 0.001);
        rotController.enableContinuousInput(-180, 180);

        double rotate = rotController.calculate(s_Swerve.gyro.getYaw(), s_Swerve.getYaw().getDegrees() + 15*s_limelight.getRX());

        /* Drive */
        s_Swerve.drive(
            new Translation2d(translationVal, strafeVal).times(Constants.Swerve.maxSpeed), 
            -rotate,
            !robotCentricSup.getAsBoolean(), 
            true
        );
    }
 
}
