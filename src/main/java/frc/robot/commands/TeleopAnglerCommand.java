package frc.robot.commands;

import java.util.function.DoubleSupplier;
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants;
import frc.robot.subsystems.Angler;

public class TeleopAnglerCommand extends Command {
    private final Angler angler;
    private final DoubleSupplier motionSup;

    public TeleopAnglerCommand(Angler angler, DoubleSupplier motionSup) {
        this.angler = angler;
        this.motionSup = motionSup;
        addRequirements(this.angler);
    }

    @Override
    public void execute() {
        /* Apply Deadband */
        double axisValue = MathUtil.applyDeadband(motionSup.getAsDouble(), Constants.STICK_DEADBAND);

        /* Move Angler */
        if (axisValue != 0.0) {
            angler.moveAngle(axisValue);
        } else {
            angler.holdTarget();
        }
    }
}
