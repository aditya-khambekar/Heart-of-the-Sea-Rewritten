package org.team4639._util;

import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;

public class DriverStationUtil {
    private static final Alliance DEFAULT_ALLIANCE = Alliance.Blue;

    public static Alliance getAlliance() {
        return DriverStation.getAlliance().orElse(DEFAULT_ALLIANCE);
    }

    public static boolean usingDSAlliance() {
        return DriverStation.getAlliance().isPresent();
    }
}
