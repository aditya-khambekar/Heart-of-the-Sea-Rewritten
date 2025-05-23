package org.team4639.lib.io.sensor.lasercan;

import au.grapplerobotics.ConfigurationFailedException;
import au.grapplerobotics.LaserCan;
import au.grapplerobotics.interfaces.LaserCanInterface;
import au.grapplerobotics.simulation.MockLaserCan;
import edu.wpi.first.units.Units;
import edu.wpi.first.units.measure.Distance;

import java.util.function.Supplier;

public class LaserCanIOSim extends LaserCanIO {
    private final MockLaserCan sensor;
    private final Supplier<Distance> measurementSupplier;


    public LaserCanIOSim(Supplier<Distance> measurementSupplier) {
        this.sensor = new MockLaserCan();
        this.measurementSupplier = measurementSupplier;
    }
    @Override
    public void updateInputs(LaserCanIOInputs inputs) {
        sensor.setMeasurementPartialSim(LaserCan.LASERCAN_STATUS_VALID_MEASUREMENT, (int) measurementSupplier.get().in(Units.Millimeters), 0);
        inputs.measurement = Units.Millimeters.of(sensor.getMeasurement().distance_mm);
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
