package frc.robot.subsystems;
import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class Carriage extends SubsystemBase {
    
    private CANSparkMax carriage;
    DigitalInput carriageBeam = new DigitalInput(1);    

    public Carriage() {
        carriage = new CANSparkMax(Constants.Carriage.CARRIAGE_ID, MotorType.kBrushless);
        carriage.restoreFactoryDefaults();
        carriage.setInverted(true);
    }

    public void setSpeed(double speed) {
        carriage.set(speed);
    }

    public double getSpeed() {
        return carriage.get();
    }

    public boolean getBeamBroken() {
        return carriageBeam.get();
    }
}
