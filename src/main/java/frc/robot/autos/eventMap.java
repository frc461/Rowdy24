package frc.robot.autos;

import java.util.HashMap;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.robot.Constants;
import frc.robot.subsystems.Elevator;
import frc.robot.subsystems.Swerve;

public class eventMap {
    public HashMap<String, Command> eventMap = new HashMap<String, Command>();
    private final Swerve s_Swerve;

    private final Elevator s_Elevator;

    public eventMap(Swerve _s_Swerve, Elevator _s_Elevator) {
        this.s_Swerve = _s_Swerve;
        this.s_Elevator = _s_Elevator;

        eventMap.put("balance", new InstantCommand(() -> s_Swerve.autoBalance())); //was just autoBalance *** balance\ on charge station
        eventMap.put("elevatorUp", new InstantCommand(() -> s_Elevator.setHeight(Constants.elevatorHighScore)));
        eventMap.put("autoCorrect", new InstantCommand(() -> s_Swerve.rotateToDegree(180)));
        

    }

    public HashMap<String, Command> getMap() {
        return eventMap;
    }

}
