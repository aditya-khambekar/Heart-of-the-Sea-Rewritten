package org.team4639.lib.io.sensor.lasercan;

import au.grapplerobotics.ConfigurationFailedException;
import au.grapplerobotics.interfaces.LaserCanInterface;
import edu.wpi.first.units.measure.Distance;
import org.littletonrobotics.junction.AutoLog;

public abstract class LaserCanIO {

    @AutoLog
    public class LaserCanIOInputs{
        Distance measurement;
        int status;
    }

    public abstract void updateInputs(LaserCanIOInputs inputs);

    public abstract void setRangingMode(LaserCanInterface.RangingMode rangingMode) throws ConfigurationFailedException;

    public abstract void setRegionOfInterest(LaserCanInterface.RegionOfInterest regionOfInterest) throws ConfigurationFailedException;
}
