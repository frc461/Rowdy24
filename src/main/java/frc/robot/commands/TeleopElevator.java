package frc.robot.commands;

import java.util.function.DoubleSupplier;
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants;
import frc.robot.subsystems.Elevator;

public class TeleopElevator extends Command {
    private final Elevator elevator;
    private final DoubleSupplier motionSup;

    public TeleopElevator(Elevator elevator, DoubleSupplier motionSup) {
        this.elevator = elevator;
        addRequirements(elevator);
        this.motionSup = motionSup;
    }

    @Override
    public void execute() {
        /* Get Values, Deadband */
        double axisValue = MathUtil.applyDeadband(motionSup.getAsDouble(), Constants.STICK_DEADBAND);

        /* Check for limit switch */
        elevator.checkLimitSwitches();

        /* Drive */
        if (axisValue != 0) {
            elevator.moveElevator(axisValue);
        } else {
            elevator.holdHeight();
        }
    }
}
