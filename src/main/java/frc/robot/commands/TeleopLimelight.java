package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.Limelight;

public class TeleopLimelight extends Command {
    private Limelight s_Limelight;
 // private DoubleSupplier vec;
  /** Creates a new TeleopRoller. */
  public TeleopLimelight(Limelight s_Limelight) {
    this.s_Limelight = s_Limelight;
    addRequirements(s_Limelight); 
  // Called when the command is initially scheduled.
}

// Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    s_Limelight.refreshValues();
  }
    
}
