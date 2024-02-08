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
    public void execute() 
    {
       angler.setAngle(36.3 * Math.pow(limelight.getRZ(), -1.17));
    }

}