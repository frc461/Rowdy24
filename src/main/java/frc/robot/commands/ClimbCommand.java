package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.Elevator;

public class ClimbCommand extends Command {
    Elevator elevator;
    
    public ClimbCommand(Elevator elevator) {
        this.elevator = elevator;
        addRequirements(elevator);
    }

    @Override
    public void execute() {
        elevator.climb();
    }

    @Override
    public void end(boolean isFinished) {
        if (isFinished) {
            elevator.climb(); // climb function will finally detect the limit switch and set the clamp to true
        }
    }


}
