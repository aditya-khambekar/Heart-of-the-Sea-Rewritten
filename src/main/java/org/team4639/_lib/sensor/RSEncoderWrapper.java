package org.team4639._lib.sensor;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import java.util.function.DoubleSupplier;
import org.littletonrobotics.junction.AutoLogOutput;
import org.team4639._lib.subsystem.Updatable;

/**
 * Wraps an encoder and provides logging and debug functionality as well as utility methods for
 * scaling values and setting offsets.
 */
public class RSEncoderWrapper implements Updatable {
  protected DoubleSupplier encoderMeasurementSupplier;
  protected String name;

  protected double offset;
  protected double multiplier;
  protected double value;

  public RSEncoderWrapper(DoubleSupplier encoderMeasurementSupplier) {
    this.encoderMeasurementSupplier = encoderMeasurementSupplier;
    this.name = "RSEncoderWrapper" + this;
    this.offset = 0.0;
    this.multiplier = 1.0;
  }

  public void setName(String name) {
    this.name = name;
  }

  public RSEncoderWrapper withName(String name) {
    this.name = name;
    return this;
  }

  public void update() {
    value = multiplier * (encoderMeasurementSupplier.getAsDouble() + offset);
    SmartDashboard.putNumber(name + "/value", value);
    SmartDashboard.putNumber(name + "/raw value", encoderMeasurementSupplier.getAsDouble());
    SmartDashboard.putNumber(name + "/offset", offset);
    SmartDashboard.putNumber(name + "/multiplier", multiplier);
  }

  public String getName() {
    return name;
  }

  @AutoLogOutput(key = "{name}Value")
  public double getValue() {
    return value;
  }

  @AutoLogOutput(key = "{name}Offset")
  public double getOffset() {
    return offset;
  }

  @AutoLogOutput(key = "{name}Multiplier")
  public double getMultiplier() {
    return multiplier;
  }

  public void setMultiplier(double multiplier) {
    this.multiplier = multiplier;
  }

  public void setOffset(double offset) {
    this.offset = offset;
  }

  /**
   * Manipulates the offset such that the current value of the raw encoder measurement would return
   * 0.
   */
  public void zero() {
    offset = offset - value;
  }
}
