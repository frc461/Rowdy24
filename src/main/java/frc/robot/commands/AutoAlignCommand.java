package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.Angler;
import frc.robot.subsystems.Limelight;

public class AutoAlignCommand extends Command {
    Angler angler;
    double x;
    double z;
    boolean tagExists;

    @SuppressWarnings("unused")
    public AutoAlignCommand(Angler angler) {
        this.angler = angler;
        this.x = Limelight.getRX();
        this.z = Limelight.getRX();
        this.tagExists = Limelight.tagExists();
        addRequirements(angler);
    }

    @Override
    public void execute() {
        x = Limelight.getRX();
        z = Limelight.getRZ();
        tagExists = Limelight.tagExists();
        angler.setAlignedAngle(x, z, tagExists);
    }
}
