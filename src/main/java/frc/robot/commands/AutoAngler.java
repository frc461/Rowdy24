package frc.robot.commands;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.Limelight;
import frc.robot.subsystems.Angler;

public class AutoAngler extends Command {

    private final Angler angler;
    private final Limelight limelight;

    public AutoAngler(Angler angler, Limelight limelight) {
        this.angler = angler;
        this.limelight = limelight;
        addRequirements(angler, limelight);
    }

    @Override
    public void execute() {
        double z = limelight.getRZ();
        double x = limelight.getRX();
        double dist = Math.pow(Math.pow(z, 2) + Math.pow(x, 2), 0.5);
        if (z < 2.38) {
            angler.setAngle(Math.min(43.9 * Math.pow(dist, -1.3), 20));
        } else {
            angler.setAngle(Math.min(49.8*Math.pow(dist, -1.31), 20));
        }
    }
}