package frc.robot.subsystems;

import com.revrobotics.CANSparkLowLevel.MotorType;
import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.SparkAbsoluteEncoder;
import com.revrobotics.SparkLimitSwitch;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class Angler extends SubsystemBase {
    private final CANSparkMax shooterAngler;
    //private final SparkPIDController anglerPID;
    private double target;
    private double position;

    private PIDController pidController;
    private RelativeEncoder neoEncoder;


    @Override
    public void periodic() {
       // if(lowerSwitchTriggered()){neoEncoder.setPosition(0);}
        
    }
    public Angler() {
        shooterAngler = new CANSparkMax(Constants.Shooter.ANGLER_ID, MotorType.kBrushless);
        //shooterAngler.restoreFactoryDefaults();
        shooterAngler.setSmartCurrentLimit(Constants.Shooter.ANGLER_CURRENT_LIMIT);
        shooterAngler.setInverted(Constants.Shooter.ANGLER_INVERT); 
        // anglerPID = shooterAngler.getPIDController();
        // anglerPID.setP(Constants.Shooter.ANGLER_P);
        // anglerPID.setI(Constants.Shooter.ANGLER_I);
        // anglerPID.setD(Constants.Shooter.ANGLER_D);

        pidController = new PIDController(0, 0, 0);

        pidController.setP(Constants.Shooter.ANGLER_P);
        pidController.setI(Constants.Shooter.ANGLER_I);
        pidController.setD(Constants.Shooter.ANGLER_D);

        neoEncoder = shooterAngler.getEncoder();
        neoEncoder.setPosition(0);
        while(!lowerSwitchTriggered()){
            shooterAngler.set(-0.5);
        }

        //anglerPID.setFeedbackDevice(neoEncoder);

        target = 0;//Constants.Shooter.ANGLER_UPPER_LIMIT;
    }


    public boolean lowerSwitchTriggered(){
        return shooterAngler.getReverseLimitSwitch(SparkLimitSwitch.Type.kNormallyOpen).isPressed();
    }

    public boolean upperSwitchTriggered(){
        return shooterAngler.getForwardLimitSwitch(SparkLimitSwitch.Type.kNormallyOpen).isPressed();
    }

    public void checkLimitSwitch(){
        if(upperSwitchTriggered()){neoEncoder.setPosition(Constants.Shooter.ANGLER_UPPER_LIMIT);}
        if(lowerSwitchTriggered()){neoEncoder.setPosition(Constants.Shooter.ANGLER_LOWER_LIMIT);}
    }

    public void holdTilt() {
        shooterAngler.set(pidController.calculate(neoEncoder.getPosition(), target));
        SmartDashboard.putNumber("Angler PID Power", pidController.calculate(neoEncoder.getPosition(), target));
        //setTolerance
    }

    public void moveAngle(double axisValue) {
        target = neoEncoder.getPosition();
        if (axisValue < 0 && lowerSwitchTriggered()){
            target = Constants.Shooter.ANGLER_LOWER_LIMIT;
            holdTilt();
        } else if (axisValue > 0 && upperSwitchTriggered()) {
            target = Constants.Shooter.ANGLER_UPPER_LIMIT;
            holdTilt();
        }else{
            shooterAngler.set(axisValue);
        }
    }

    public void setAngle(double angle) {
        if (angle < neoEncoder.getPosition() && lowerSwitchTriggered()) {
            neoEncoder.setPosition(Constants.Shooter.ANGLER_LOWER_LIMIT);
            angle = Constants.Shooter.ANGLER_LOWER_LIMIT;
        } else if (angle > neoEncoder.getPosition() && neoEncoder.getPosition() > Constants.Shooter.ANGLER_UPPER_LIMIT) {
            angle = Constants.Shooter.ANGLER_UPPER_LIMIT;
        }
        shooterAngler.set(pidController.calculate(neoEncoder.getPosition(), angle));
        target = angle;
    }
    public double getEncoder() {
        return  neoEncoder.getPosition(); 
        
        
        //anglerEncoder.getPosition();
    }
}
