package frc.robot.commands;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.Intake;

public class TeleopIntake extends Command {
  private Intake s_Intake;
  private Joystick operator;
  
  /** Creates a new TeleopRoller. */
  public TeleopIntake(Intake s_Intake, Joystick operator) {
    this.s_Intake = s_Intake;
    this.operator = operator;
    addRequirements(s_Intake); 
  }

  @Override
  public void execute() {
    s_Intake.runIntake(operator);
  }

}
