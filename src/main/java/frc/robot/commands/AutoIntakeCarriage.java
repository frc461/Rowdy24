package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.IntakeCarriage;

public class AutoIntakeCarriage extends Command {

    private final IntakeCarriage intakeCarriage;

    public AutoIntakeCarriage(IntakeCarriage intakeCarriage) {
        this.intakeCarriage = intakeCarriage;
        addRequirements(intakeCarriage);
    }

    @Override
    public void execute() {
        intakeCarriage.setIntakeSpeed(0.75);
        intakeCarriage.setCarriageSpeed(0.75);
    }

}
