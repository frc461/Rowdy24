package frc.robot.constants;

public enum RobotIdentity {
    PRACTICE,
    ROWDY;

    public static RobotIdentity getIdentity() {
        String mac = MacAddress.getMACAddress();
        if (!mac.isEmpty()) {
            if (mac.equals(MacAddress.PRACTICE)) {
                return PRACTICE;
            }
            else if (mac.equals(MacAddress.ROWDY)) {
                return ROWDY;
            }
        }
        return ROWDY;
    }
}
