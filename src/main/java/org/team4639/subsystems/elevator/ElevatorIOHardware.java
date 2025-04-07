package org.team4639.subsystems.elevator;

import java.util.function.DoubleSupplier;

public class ElevatorIOHardware implements ElevatorIO {
  private DoubleSupplier speedSupplier = () -> 0.0;
  private DoubleSupplier positionSupplier = () -> 0.0;

  @Override
  public void updateInputs(ElevatorIOInputs inputs) {
    inputs.encoderMeasurement = positionSupplier.getAsDouble();
    inputs.encoderSpeed = speedSupplier.getAsDouble();
  }

  @Override
  public void sendTalonInputs(DoubleSupplier speedSupplier, DoubleSupplier positionSupplier) {
    this.speedSupplier = speedSupplier;
    this.positionSupplier = positionSupplier;
  }
}
