package org.team4639.subsystems.elevator;

import java.util.function.DoubleSupplier;
import org.littletonrobotics.junction.AutoLog;

public interface ElevatorIO {
  @AutoLog
  class ElevatorIOInputs {
    double encoderMeasurement = 0.0;
    double encoderSpeed = 0.0;
  }

  default void updateInputs(ElevatorIOInputs inputs) {}

  /**
   * Only used in {@link ElevatorIOHardware} because it pulls inputs from the TalonFX motors that
   * live in the main {@link Elevator} subsystem. This should be called in the constructor of the
   * elevator system to the ElevatorIO objects that are passed into it.
   *
   * @param speedSupplier supplies the speed of the motor.
   * @param positionSupplier supplies the position of the motor.
   */
  default void sendTalonInputs(DoubleSupplier speedSupplier, DoubleSupplier positionSupplier) {}
}
