package frc.robot.commands;

import java.util.function.DoubleSupplier;
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants;
import frc.robot.subsystems.Angler;

public class TeleopAngler extends Command {
    private final Angler angler;
    private final DoubleSupplier motionSup;

    public TeleopAngler(Angler angler, DoubleSupplier motionSup) {
        this.angler = angler;
        this.motionSup = motionSup;
        addRequirements(angler);
    }

    @Override
    public void execute() {
        /* Apply Deadband */
        double axisValue = MathUtil.applyDeadband(motionSup.getAsDouble(), Constants.STICK_DEADBAND);

        /* Check Limit Switches */
        angler.checkLimitSwitches();

        /* Move Angler */
        if (axisValue != 0.0) {
            angler.moveAngle(axisValue);
        } else {
            angler.holdTilt();
        }
    }
}
