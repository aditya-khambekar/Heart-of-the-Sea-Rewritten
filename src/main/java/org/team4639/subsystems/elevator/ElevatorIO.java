package org.team4639.subsystems.elevator;

import org.littletonrobotics.junction.AutoLog;

public interface ElevatorIO {
  @AutoLog
  class ElevatorIOInputs {
    double encoderMeasurement = 0.0;
    double encoderSpeed = 0.0;
  }

  default void updateInputs(ElevatorIOInputs inputs) {}

  default void setMotionMagicPosition(double setpointEncoder) {}

  default void setVelocityControl(double velocityRPS){}
}
