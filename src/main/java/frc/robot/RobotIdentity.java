package frc.robot;

public enum RobotIdentity {
    ALPHA_BOT,
    BETA_BOT;

    public static RobotIdentity getIdentity() {
        String mac = MacAddress.getMACAddress();
        if (!mac.equals("")) {
            if (mac.equals(MacAddress.AlphaBot)) {
                return ALPHA_BOT;
            }
            else if (mac.equals(MacAddress.BetaBot)) {
                return BETA_BOT;
            }
        }
        return ALPHA_BOT;
    }
}


