package org.team4639.lib.io.sensor.lasercan;

import au.grapplerobotics.ConfigurationFailedException;
import au.grapplerobotics.LaserCan;
import au.grapplerobotics.interfaces.LaserCanInterface;
import edu.wpi.first.units.Units;

public class LaserCanIOHardware extends LaserCanIO {
    private LaserCan sensor;

    public LaserCanIOHardware(int deviceID) {
        sensor = new LaserCan(deviceID);
    }

    @Override
    public void updateInputs(LaserCanIOInputs inputs) {
        inputs.measurement = Units.Millimeter.of(sensor.getMeasurement().distance_mm);
        inputs.status = sensor.getMeasurement().status;
    }

    @Override
    public void setRangingMode(LaserCanInterface.RangingMode rangingMode) throws ConfigurationFailedException {
        sensor.setRangingMode(rangingMode);
    }

    @Override
    public void setRegionOfInterest(LaserCanInterface.RegionOfInterest regionOfInterest) throws ConfigurationFailedException {
        sensor.setRegionOfInterest(regionOfInterest);
    }
}
