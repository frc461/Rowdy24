
package frc.robot.autos;
import com.pathplanner.lib.auto.AutoBuilder;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj2.command.Command;

public class AutoChooser {
    private final SendableChooser<AutonomousMode> m_chooser= new SendableChooser<>();
    private PIDController thetaController = new PIDController(0.05, 0.005, 0.009);


    public AutoChooser() {
        m_chooser.addOption("default", AutonomousMode.kDefaultAuto);
    }

    public SendableChooser<AutonomousMode> getAutoChooser() {
        return m_chooser;
    }

    public PIDController getPIDController() {
        return thetaController;
    }

    public Command defaultAuto() {
        return AutoBuilder.buildAuto("New Auto");

    }

    public Command getCommand() {
        switch (m_chooser.getSelected()) {
            case kDefaultAuto :
            return defaultAuto();
        }
        return defaultAuto();
    }


    private enum AutonomousMode {
        kDefaultAuto, kAudienceAuto, kCenterAuto, kTwoGameP, kCollectBalanceAud,
        kCollectBalanceScore, kScoreMobilityEngage, kScoremobilityengagepickup,
        kScoremobilitycollect, kScoremobilitycollectcablecarrier, kTwoPiece, kThreePiece,
        kTwoCube, kTwoCubeCC, alternatePickup, kThreeLow
    }

    

}