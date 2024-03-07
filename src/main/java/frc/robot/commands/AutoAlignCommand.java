package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.Angler;
import frc.robot.subsystems.Limelight;

public class AutoAlignCommand extends Command {
    Angler angler;

    @SuppressWarnings("unused")
    public AutoAlignCommand(Angler angler) {
        this.angler = angler;
        addRequirements(angler);
    }

    @Override
    public void execute() {
        angler.setAlignedAngle();
    }
}
