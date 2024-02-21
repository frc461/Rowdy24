package frc.robot.constants;

import frc.robot.constants.variants.PracticeConstants;
import frc.robot.constants.variants.RowdyConstants;

public interface RobotConstants {
    Configuration getConfiguration();

    static RobotConstants getRobotConstants(RobotIdentity robot) {
        switch (robot) {
            case PRACTICE:
                return new PracticeConstants();
            case ROWDY:
                return new RowdyConstants();
            default:
                return new PracticeConstants();
        }
    }
}
