package org.team4639.subsystems.scoring;

import com.revrobotics.spark.SparkMax;

public class ScoringIOHardware implements ScoringIO {
  SparkMax spark;

  public ScoringIOHardware(SparkMax spark) {
    this.spark = spark;
  }

  public void updateInputs(ScoringIOInputs inputs) {
    if (spark != null) {
      inputs.currentAmps = spark.getOutputCurrent();
    }
  }

  public void runSparkMax(double speed) {
    spark.set(speed);
  }
}
