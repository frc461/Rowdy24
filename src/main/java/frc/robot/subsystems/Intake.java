package frc.robot.subsystems;
import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class Intake extends SubsystemBase{
    private CANSparkMax intake;
    DigitalInput intakeBeam = new DigitalInput(0);    
    public boolean intakeCube = false;

    public Intake() {
        intake = new CANSparkMax(Constants.Intake.INTAKE_ID, MotorType.kBrushless);
        intake.restoreFactoryDefaults();
        intake.setInverted(true);
    }

    public void setSpeed(double speed) {
        // TODO: check if a piece is already in the intake
        intake.set(speed);
    }

    public double getSpeed() {
        return intake.get();
    }

    public boolean getBeamBroken() {
        return intakeBeam.get();
    }
}
