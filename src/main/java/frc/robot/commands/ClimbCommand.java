package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.Elevator;

public class ClimbCommand extends Command {

    Elevator elevator;
    public ClimbCommand(Elevator elevator) {
        
        this.elevator = elevator;

    }

    @Override
    public void execute() {
        elevator.climb();
    }

    @Override
    public void end(boolean isFinished) {
        if (isFinished) {
            elevator.setClamp();
        }
    }


}
