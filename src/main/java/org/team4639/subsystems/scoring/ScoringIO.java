package org.team4639.subsystems.scoring;

import com.revrobotics.spark.SparkMax;
import org.littletonrobotics.junction.AutoLog;

public interface ScoringIO {

  @AutoLog
  class ScoringIOInputs {
    double currentAmps;
  }

  default void updateInputs(ScoringIOInputs inputs) {}

  default void sendSparkMaxData(SparkMax spark) {}
}
