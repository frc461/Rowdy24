package frc.robot.subsystems;
import com.revrobotics.CANSparkMax;
import com.revrobotics.SparkMaxAbsoluteEncoder;
import com.revrobotics.CANSparkLowLevel.MotorType;
import edu.wpi.first.wpilibj.DigitalInput;
import com.revrobotics.CANSparkFlex;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class IntakeCarriage extends SubsystemBase{
    private final CANSparkFlex intake;
    private final CANSparkMax carriage;
    
    //TODO: ask tech to put beam breaks in these places
    DigitalInput carriageBeam = new DigitalInput(0); // end of carriage (on shooter side)
    DigitalInput ampBeam = new DigitalInput(1); // entrance of carriage (which is the amp shooter)
    DigitalInput shooterBeam = new DigitalInput(2); // completely exit through shooter
    
    private boolean hasPiece = false;

    public IntakeCarriage() {
        intake = new CANSparkFlex(Constants.Intake.INTAKE_ID, MotorType.kBrushless);

        intake.restoreFactoryDefaults();
        intake.setSmartCurrentLimit(50);
        intake.setInverted(false);

        carriage = new CANSparkMax(Constants.Carriage.CARRIAGE_ID, MotorType.kBrushless);
        carriage.restoreFactoryDefaults();
        carriage.setInverted(true);
    }

    @Override
    public void periodic() {
        // if (getCarriageBeamBroken()) {
        //     hasPiece = true;
        // } else if (getAmpBeamBroken() || getShooterBeamBroken()) {
        //     hasPiece = false;
        // }
    }

    public double getIntakeSpeed() {
        return intake.get();
    }

    public double getCarriageSpeed() {
        return carriage.get();
    }

    public boolean getAmpBeamBroken() {
        return !ampBeam.get();
    }

    public boolean getShooterBeamBroken() {
        return !shooterBeam.get();
    }

    public boolean getCarriageBeamBroken() {
        return !carriageBeam.get();
    }

    public void setIntakeSpeed(double speed) {
        if (speed <= 0) {
            intake.set(speed);
        } else if (!hasPiece) {
            intake.set(speed);
        } else {
            intake.set(-1);
        }
    }

    public void setCarriageSpeed(double speed) {
        carriage.set(speed);
    }
}
