package frc.robot.subsystems;
import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class Intake extends SubsystemBase{
    private CANSparkMax intake;
    
    private CANSparkMax carriage;
    
    //TODO: ask tech to put beam breaks in these places
    DigitalInput carriageBeam = new DigitalInput(0); // enter the carriage (completely out of the intake)
    DigitalInput ampBeam = new DigitalInput(1); // exit to amp
    DigitalInput shooterBeam = new DigitalInput(2); // exit through shooter
    
    private boolean hasPiece = false;

    public Intake() {
        intake = new CANSparkMax(Constants.Intake.INTAKE_ID, MotorType.kBrushed);
        intake.restoreFactoryDefaults();
        intake.setInverted(true);

        carriage = new CANSparkMax(Constants.Carriage.CARRIAGE_ID, MotorType.kBrushless);
        carriage.restoreFactoryDefaults();
        carriage.setInverted(true);
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

    public boolean getAmpBeamBroken() {
        return ampBeam.get();
    }

    public boolean getShooterBeamBroken() {
        return shooterBeam.get();
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
