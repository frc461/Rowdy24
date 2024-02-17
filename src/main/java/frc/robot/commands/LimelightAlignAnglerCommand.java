package frc.robot.commands;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.Limelight;
import frc.robot.subsystems.Angler;

public class LimelightAlignAnglerCommand extends Command {

    private final Angler angler;
    private final Limelight limelight;

    public LimelightAlignAnglerCommand(Angler angler, Limelight limelight) {
        this.angler = angler;
        this.limelight = limelight;
        addRequirements(this.angler, this.limelight);
    }

    @Override
    public void initialize() {
        angler.setAlignedAngle(limelight.getRX(), limelight.getRZ(), limelight.tagExists());
    }

    @Override
    public boolean isFinished() {
        return true;
    }
}