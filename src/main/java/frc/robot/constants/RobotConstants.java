package frc.robot.constants;

import frc.robot.constants.variants.PracticeConstants;
import frc.robot.constants.variants.RowdyConstants;

public interface RobotConstants {
    static Object getRobotConstants(RobotIdentity robot) {
        return switch (robot) {
            case PRACTICE -> new PracticeConstants();
            case ROWDY -> new RowdyConstants();
            default -> new RowdyConstants();
        };
    }
}
