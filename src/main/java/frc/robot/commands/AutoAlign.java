package frc.robot.commands;
import edu.wpi.first.wpilibj2.command.Command;

import frc.robot.subsystems.Limelight;
import frc.robot.subsystems.Angler;

public class AutoAlign extends Command {

    private final Angler angler;
    private final Limelight limelight;

    public AutoAlign(Angler angler, Limelight limelight) {
        this.angler = angler;
        this.limelight = limelight;
        addRequirements(angler, limelight);
    }

    @Override
    public void execute() {
        //TODO: VERIFY METHODS

        /* Calculate angler pitch-wise trajectory */

        /* Use PID to turret-aim to speaker while moving with current swerve module states */
    }
}
