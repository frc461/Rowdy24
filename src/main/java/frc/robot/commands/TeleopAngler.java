package frc.robot.commands;

import java.util.function.DoubleSupplier;
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants;
import frc.robot.subsystems.Angler;

public class TeleopAngler extends Command {
    private Angler angler;
    private DoubleSupplier motionSup;

    public TeleopAngler(Angler angler, DoubleSupplier motionSup) {
        this.angler = angler;
        this.motionSup = motionSup;
        addRequirements(angler);
    }

    @Override
    public void execute() {
        double axisValue = MathUtil.applyDeadband(motionSup.getAsDouble(), Constants.STICK_DEADBAND);

        // check limit switches
        angler.checkLimitSwitches();

        if (axisValue!= 0.0) {
            angler.moveAngle(motionSup.getAsDouble());
        } else {
            angler.holdTilt();
        }

    }
}
