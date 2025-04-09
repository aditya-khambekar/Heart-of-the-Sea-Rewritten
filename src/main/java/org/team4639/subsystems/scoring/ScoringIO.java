package org.team4639.subsystems.scoring;

import org.littletonrobotics.junction.AutoLog;

public interface ScoringIO {

  @AutoLog
  class ScoringIOInputs {
    double currentAmps;
    boolean hasCoral;
  }

  default void updateInputs(ScoringIOInputs inputs) {}

  default void runSparkMax(double speed) {}
}
