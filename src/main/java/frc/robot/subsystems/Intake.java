//package frc.robot.subsystems;
//import com.revrobotics.CANSparkMax;
//
//import com.revrobotics.CANSparkMaxLowLevel;
//import edu.wpi.first.wpilibj.AddressableLED;
//import edu.wpi.first.wpilibj.AddressableLEDBuffer;
//import edu.wpi.first.wpilibj.DigitalInput;
//import edu.wpi.first.wpilibj.DigitalOutput;
//import edu.wpi.first.wpilibj2.command.SubsystemBase;
//
//public class Intake extends SubsystemBase{
//    private CANSparkMax intake;
//    //DigitalInput cubeBeam = new DigitalInput(0);
//    //DigitalInput coneBeam = new DigitalInput(1);
//
//    private DigitalOutput intakeIndicator = new DigitalOutput(4);
//
//    public boolean intakeCube = false;
//
//    public Intake() {
//        intake = new CANSparkMax(33, CANSparkMaxLowLevel.MotorType.kBrushed);
//        intake.restoreFactoryDefaults();
//        intake.setInverted(true);
////        led.setLength(ledData.getLength());
//        //lights.showLights("blue");
//
//    }
//
//
//}
