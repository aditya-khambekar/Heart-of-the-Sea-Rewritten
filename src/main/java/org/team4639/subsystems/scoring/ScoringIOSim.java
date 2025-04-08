package org.team4639.subsystems.scoring;

import com.revrobotics.spark.SparkMax;

public class ScoringIOSim implements ScoringIO {
  SparkMax spark;
  // TODO: add DCMotorSim

  public void updateInputs(ScoringIOInputs inputs) {
    if (spark != null) {
      inputs.currentAmps = spark.getOutputCurrent();
    }
  }

  public void sendSparkMaxData(SparkMax spark) {
    this.spark = spark;
  }
}
