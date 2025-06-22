package org.team4639.lib.io.sensor.lasercan;

import static edu.wpi.first.units.Units.Millimeter;

import au.grapplerobotics.ConfigurationFailedException;
import au.grapplerobotics.LaserCan;
import au.grapplerobotics.interfaces.LaserCanInterface;
import edu.wpi.first.units.measure.Distance;
import edu.wpi.first.util.sendable.Sendable;
import edu.wpi.first.util.sendable.SendableBuilder;

public abstract class LaserCanIO implements Sendable {
  public static class LaserCanIOInputs {
    public Distance measurement = Millimeter.of(Integer.MAX_VALUE);
    public int status = LaserCan.LASERCAN_STATUS_VALID_MEASUREMENT;
    public LaserCanInterface.RangingMode mode = LaserCanInterface.RangingMode.LONG;
    public LaserCanInterface.RegionOfInterest regionOfInterest;
    public LaserCanInterface.TimingBudget timingBudget;
  }

  public abstract void updateInputs(LaserCanIOInputs inputs);

  public abstract void setRangingMode(LaserCanInterface.RangingMode rangingMode)
      throws ConfigurationFailedException;

  public abstract void setRegionOfInterest(LaserCanInterface.RegionOfInterest regionOfInterest)
      throws ConfigurationFailedException;

  public abstract void setTimingBudget(LaserCanInterface.TimingBudget timingBudget)
      throws ConfigurationFailedException;

  protected abstract double getMeasurement();

  protected abstract int getStatus();

  protected abstract LaserCanInterface.TimingBudget getTimingBudget();

  protected abstract LaserCanInterface.RangingMode getRangingMode();

  protected abstract LaserCanInterface.RegionOfInterest getRegionOfInterest();

  public void initSendable(SendableBuilder sendableBuilder) {
    sendableBuilder.addDoubleProperty("measurement_mm", this::getMeasurement, null);
    sendableBuilder.addDoubleProperty("status_mm", this::getStatus, null);
    sendableBuilder.addStringProperty(
        "Timing Budget", () -> this.getTimingBudget().toString(), null);
    sendableBuilder.addStringProperty("Ranging Mode", () -> this.getRangingMode().toString(), null);
    sendableBuilder.addStringProperty(
        "Region of Interest", () -> this.getRegionOfInterest().toString(), null);
  }
}
