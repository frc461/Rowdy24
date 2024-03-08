package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.IntakeCarriage;

public class IntakeCarriageCommand extends Command {
    private final IntakeCarriage intakeCarriage;
    private final double intakeSpeed;
    private final double carriageSpeed;

    public IntakeCarriageCommand(IntakeCarriage intakeCarriage, double intakeSpeed, double carriageSpeed) {
        this.intakeCarriage = intakeCarriage;
        this.intakeSpeed = intakeSpeed;
        this.carriageSpeed = carriageSpeed;
        addRequirements(this.intakeCarriage);
    }

    @Override
    public void initialize() {
        intakeCarriage.setIntakeCarriageSpeed(intakeSpeed, carriageSpeed);
    }

    @Override
    public void execute() {
        if (intakeCarriage.noteInShootingSystem() && intakeSpeed > 0) {
            intakeCarriage.setIntakeSpeed(0);
            intakeCarriage.setCarriageSpeed(carriageSpeed);
        } else {
            intakeCarriage.setIntakeCarriageSpeed(intakeSpeed, carriageSpeed);
        }
    }

    @Override
    public void end(boolean isFinished) {
        intakeCarriage.setIntakeSpeed(0);
        intakeCarriage.setCarriageSpeed(0);
    }
}