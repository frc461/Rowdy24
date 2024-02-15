package frc.robot.commands;

import java.util.function.DoubleSupplier;
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants;
import frc.robot.subsystems.Elevator;

public class MoveElevatorCommand extends Command {
    private final Elevator elevator;
    private final DoubleSupplier motionSup;

    public MoveElevatorCommand(Elevator elevator, DoubleSupplier motionSup) {
        this.elevator = elevator;
        this.motionSup = motionSup;
        addRequirements(elevator);
    }

    @Override
    public void execute() {
        /* Apply Deadband */
        double axisValue = MathUtil.applyDeadband(motionSup.getAsDouble(), Constants.STICK_DEADBAND);

        /* Move Elevator */
        if (axisValue != 0) {
            elevator.moveElevator(axisValue);
        } else {
            elevator.holdTarget();
        }
    }
}
