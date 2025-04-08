package org.team4639.subsystems.elevator;

import com.ctre.phoenix6.hardware.TalonFX;
import java.util.function.DoubleSupplier;

public class ElevatorIOSim implements ElevatorIO {
  private DoubleSupplier speedSupplier = () -> 0.0;
  private DoubleSupplier positionSupplier = () -> 0.0;

  @Override
  public void updateInputs(ElevatorIOInputs inputs) {
    inputs.encoderMeasurement = positionSupplier.getAsDouble();
    inputs.encoderSpeed = speedSupplier.getAsDouble();
  }

  @Override
  public void sendTalonInputs(TalonFX talon) {
    this.speedSupplier = () -> talon.getVelocity().getValueAsDouble();
    this.positionSupplier = () -> talon.getPosition().getValueAsDouble();
  }
}
