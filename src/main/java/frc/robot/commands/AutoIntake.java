package frc.robot.commands;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.IntakeCarriage;

public class AutoIntake extends Command {

    private final IntakeCarriage intake;

    public AutoIntake(IntakeCarriage intakeCarriage) {
        this.intake = intakeCarriage;
        addRequirements(intake);
    }

    @Override
    public void execute() {
        intake.setIntakeSpeed(0.75);
        intake.setCarriageSpeed(0.75);
    }

}
