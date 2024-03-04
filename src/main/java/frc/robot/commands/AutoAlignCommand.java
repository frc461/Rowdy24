package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.Angler;
import frc.robot.subsystems.Limelight;

public class AutoAlignCommand extends Command {
    Angler angler;
    Limelight limelight;
    double x;
    double z;
    boolean tagExists;

    @SuppressWarnings("unused")
    public AutoAlignCommand(Angler angler, Limelight limelight) {
        this.angler = angler;
        this.limelight = limelight;
        this.x = limelight.getRX();
        this.z = limelight.getRZ();
        this.tagExists = limelight.tagExists();
        addRequirements(angler, limelight);
    }

    @Override
    public void execute() {
        x = limelight.getRX();
        z = limelight.getRZ();
        tagExists = limelight.tagExists();
        angler.setAlignedAngle(x, z, tagExists);
    }
}
