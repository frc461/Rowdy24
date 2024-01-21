package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.Limelight;

public class TeleopLimelight extends Command {
  private Limelight s_Limelight;
  
  public TeleopLimelight(Limelight s_Limelight) {
    this.s_Limelight = s_Limelight;
    addRequirements(s_Limelight); 
  }

  @Override
  public void execute() {
    s_Limelight.refreshValues();
  }
    
}
