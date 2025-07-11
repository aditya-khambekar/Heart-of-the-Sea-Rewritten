package org.team4639.lib.network;

import edu.wpi.first.util.sendable.Sendable;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

public class SmartDashboard2 {
    /**
     * Sends data to SmartDashboard and returns the data.
     */
    public static <A extends Sendable> A putData(String key, A data){
        SmartDashboard.putData(key, data);
        return data;
    }
}
