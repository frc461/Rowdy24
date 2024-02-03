package frc.robot.commands;

import java.util.function.DoubleSupplier;
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants;
import frc.robot.subsystems.Angler;

public class TeleopAngler extends Command {
    private Angler angler;
    private DoubleSupplier motionSup;

    public TeleopAngler(Angler angler, DoubleSupplier motionSup) {
        // Use addRequirements() here to declare subsystem dependencies.
        this.angler = angler;
        this.motionSup = motionSup;
        addRequirements(angler);
    }

    // Called every time the scheduler runs while the command is scheduled.
    @Override
    public void execute() {

        angler.checkLimitSwitch();

        if (MathUtil.applyDeadband(motionSup.getAsDouble(), Constants.STICK_DEADBAND) != 0.0) {
            angler.moveAngle(motionSup.getAsDouble());
        } else {
            angler.holdTilt();
        }

    }
}
