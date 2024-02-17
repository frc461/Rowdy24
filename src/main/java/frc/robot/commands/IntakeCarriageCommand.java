package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.IntakeCarriage;

public class IntakeCarriageCommand extends Command {
    private final IntakeCarriage intakeCarriage;
    private final double intakeSpeed;
    private final double carriageSpeed;
    private final boolean idleMode;
    private boolean inCarriage;

    public IntakeCarriageCommand(IntakeCarriage intakeCarriage, double intakeSpeed, double carriageSpeed, boolean idleMode) {
        this.intakeCarriage = intakeCarriage;
        this.intakeSpeed = intakeSpeed;
        this.carriageSpeed = carriageSpeed;
        this.idleMode = idleMode;
        this.inCarriage = false;
        addRequirements(intakeCarriage);
    }

    @Override
    public void initialize() {
        intakeCarriage.setIntakeCarriageSpeed(intakeSpeed, carriageSpeed);
        if (intakeCarriage.noteInSystem()) {
            inCarriage = true;
        }
    }

    @Override
    public void execute() {
        if (intakeCarriage.noteInSystem() && intakeSpeed > 0) {
            intakeCarriage.setIntakeIdle(idleMode);
        } else {
            intakeCarriage.setIntakeCarriageSpeed(intakeSpeed, carriageSpeed);
        }
        if (intakeCarriage.noteInSystem() & !inCarriage) {
            cancel();
        }
    }

    @Override
    public void end(boolean isFinished) {
        intakeCarriage.setIntakeIdle(idleMode);
        intakeCarriage.setCarriageIdle();
    }
}
