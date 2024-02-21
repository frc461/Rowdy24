package frc.robot;

public interface RobotConstants {
    Configuration getConfiguration();

    static RobotConstants getRobotConstants(RobotIdentity robot) {
        switch (robot) {
            case ALPHA_BOT:
                return new AlphaBotConstants();
            case BETA_BOT:
                return new BetaBotConstants();
            default:
                return new AlphaBotConstants();
        }
    }
}
