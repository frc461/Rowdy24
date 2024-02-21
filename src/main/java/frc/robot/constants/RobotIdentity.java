package frc.robot.constants;

public enum RobotIdentity {
    PRACTICE,
    ROWDY;

    public static RobotIdentity getIdentity() {
        String mac = MacAddress.getMACAddress();
        if (!mac.equals("")) {
            if (mac.equals(MacAddress.AlphaBot)) {
                return PRACTICE;
            }
            else if (mac.equals(MacAddress.BetaBot)) {
                return ROWDY;
            }
        }
        return ROWDY;
    }
}
