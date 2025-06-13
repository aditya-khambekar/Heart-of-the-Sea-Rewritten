package org.team4639.lib.io.sensor.lasercan;

import au.grapplerobotics.ConfigurationFailedException;
import au.grapplerobotics.LaserCan;
import au.grapplerobotics.interfaces.LaserCanInterface;
import edu.wpi.first.units.Units;

public class LaserCanIOHardware extends LaserCanIO {
  private LaserCan sensor;

  // cache last measurement
  private LaserCanInterface.Measurement measurement;

  private LaserCanInterface.RegionOfInterest regionOfInterest;
  private LaserCanInterface.TimingBudget timingBudget;
  private LaserCanInterface.RangingMode rangingMode;

  public LaserCanIOHardware(int deviceID) {
    sensor = new LaserCan(deviceID);
  }

  @Override
  public void updateInputs(LaserCanIOInputs inputs) {
    this.measurement = sensor.getMeasurement();
    inputs.measurement = Units.Millimeter.of(measurement.distance_mm);
    inputs.status = measurement.status;
  }

  @Override
  public void setRangingMode(LaserCanInterface.RangingMode rangingMode)
      throws ConfigurationFailedException {
    sensor.setRangingMode(this.rangingMode = rangingMode);
  }

  @Override
  public void setRegionOfInterest(LaserCanInterface.RegionOfInterest regionOfInterest)
      throws ConfigurationFailedException {
    sensor.setRegionOfInterest(this.regionOfInterest = regionOfInterest);
  }

  @Override
  public void setTimingBudget(LaserCanInterface.TimingBudget timingBudget)
      throws ConfigurationFailedException {
    sensor.setTimingBudget(this.timingBudget = timingBudget);
  }

  @Override
  protected double getMeasurement() {
    return measurement.distance_mm;
  }

  @Override
  protected int getStatus() {
    return measurement.status;
  }

  @Override
  protected LaserCanInterface.TimingBudget getTimingBudget() {
    return this.timingBudget;
  }

  @Override
  protected LaserCanInterface.RangingMode getRangingMode() {
    return this.rangingMode;
  }

  @Override
  protected LaserCanInterface.RegionOfInterest getRegionOfInterest() {
    return this.regionOfInterest;
  }
}
