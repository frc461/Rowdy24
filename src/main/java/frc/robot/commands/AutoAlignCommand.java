package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants;
import frc.robot.subsystems.Angler;
import frc.robot.subsystems.Swerve;

public class AutoAlignCommand extends Command {
    Angler angler;
    Swerve swerve;

    public AutoAlignCommand(Angler angler, Swerve swerve) {
        this.angler = angler;
        this.swerve = swerve;
        addRequirements(angler);
    }

    @Override
    public void execute() {
        angler.setEncoderVal(
                Constants.Angler.AUTO_ANGLER_AIM_EQUATION.apply(
                        Math.abs(swerve.getVectorToSpeakerTarget().getY()),
                        Math.abs(swerve.getVectorToSpeakerTarget().getX())
                )
        );
    }

    @Override
    public void end(boolean isFinished) {
        angler.setEncoderVal(Constants.Angler.ANGLER_LOWER_LIMIT);
    }
}
