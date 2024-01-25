package frc.robot.subsystems;
import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class Intake extends SubsystemBase{
    private CANSparkMax intake;
    DigitalInput intakeBeam = new DigitalInput(0);   
    
    private CANSparkMax carriage;
    DigitalInput carriageBeam = new DigitalInput(1);
    DigitalInput ampBeam = new DigitalInput(2);
    
    private boolean hasPiece = false;

    public Intake() {
        intake = new CANSparkMax(Constants.Intake.INTAKE_ID, MotorType.kBrushless);
        intake.restoreFactoryDefaults();
        intake.setInverted(true);

        carriage = new CANSparkMax(Constants.Carriage.CARRIAGE_ID, MotorType.kBrushless);
        carriage.restoreFactoryDefaults();
        carriage.setInverted(true);
    }

    public void setIntakeSpeed(double speed) {
        // TODO: check if a piece is already in the intake
        if (!hasPiece) {
            intake.set(speed);
        } else {
            intake.set(0);
        }
    }

    @Override
    public void periodic() {
        if (getCarriageBeamBroken()) {
            hasPiece = true;
        } else if (getAmpBeamBroken()) {
            hasPiece = false;
        }
    }

    public double getIntakeSpeed() {
        return intake.get();
    }

    public boolean getIntakeBeamBroken() {
        return intakeBeam.get();
    }

    public boolean getAmpBeamBroken() {
        return ampBeam.get();
    }

    public void setCarriageSpeed(double speed) {
        carriage.set(speed);
    }

    public double getCarriageSpeed() {
        return carriage.get();
    }

    public boolean getCarriageBeamBroken() {
        return carriageBeam.get();
    }
}
