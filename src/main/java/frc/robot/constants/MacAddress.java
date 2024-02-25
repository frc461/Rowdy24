package frc.robot.constants;

import java.net.NetworkInterface;
import java.util.Enumeration;

public class MacAddress {
    public static final String PRACTICE = "00-80-2F-17-81-C0";
    public static final String ROWDY = "00-80-2F-33-9F-37";

    public static String getMACAddress() {
        try {
            Enumeration<NetworkInterface> networkInterface = NetworkInterface.getNetworkInterfaces();
            StringBuilder macAddress = new StringBuilder();
            while (networkInterface.hasMoreElements()) {
                NetworkInterface tempInterface = networkInterface.nextElement();
                if (tempInterface != null) {
                    byte[] mac = tempInterface.getHardwareAddress();
                    if (mac != null) {
                        for (int i = 0; i < mac.length; i++) {
                            macAddress.append(String.format("%02X%s", mac[i], (i < mac.length - 1) ? "-" : ""));
                        }
                        return macAddress.toString();
                    }
                    else {
                        System.out.println("Address not accessible");
                    }
                }
                else {
                    System.out.println("Network Interface for specified address not found");
                }
            }
        }
        catch (Exception e) {
            e.printStackTrace();
        }
        return "";
    }
}
