package frc.robot.commands;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.Intake;

public class AutoIntakeCommand extends Command {
  /** Creates a new AutoIntakeCmd. */
  private Intake s_Intake;
  private final double speed;
  private final boolean cone;
  private final Timer timer;

  public AutoIntakeCommand(Intake s_Intake, double speed, boolean cone) {
    this.s_Intake = s_Intake;
    this.speed = speed;
    this.cone = cone;
    this.timer = new Timer();
    addRequirements(s_Intake);
  }

  @Override
  public void initialize() {
    System.out.println("Auto Intake Started");
    timer.start(); 
  }

  @Override
  public void execute() {
    s_Intake.setSpeed(speed);
    System.out.println(timer.get());
  }

  @Override
  public void end(boolean interrupted) {
    s_Intake.setSpeed(0);
    System.out.println("Auto Intake Ended");
    timer.reset();
  }

  @Override
  public boolean isFinished() {
    if (cone) {
      if (speed < 0) {
        return s_Intake.coneBeamBroken();
      }
      return !s_Intake.coneBeamBroken();
    }
    if (speed > 0) {
      return (s_Intake.cubeBeamBroken() && s_Intake.coneBeamBroken());
    }
    return !(s_Intake.cubeBeamBroken() || s_Intake.coneBeamBroken());
  }
}
