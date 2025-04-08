package org.team4639.subsystems.scoring;

import com.revrobotics.sim.SparkMaxSim;
import com.revrobotics.spark.SparkMax;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.wpilibj.simulation.RoboRioSim;

public class ScoringIOSim implements ScoringIO {
  SparkMax spark;
  SparkMaxSim sim;

  public ScoringIOSim(SparkMax spark) {
    this.spark = spark;
    sim = new SparkMaxSim(spark, DCMotor.getNeoVortex(1));
  }

  public void updateInputs(ScoringIOInputs inputs) {
    if (spark != null) {
      inputs.currentAmps = sim.getMotorCurrent();
    }
  }

  public void runSparkMax(double speed) {
    spark.set(speed);
    sim.iterate(spark.get(), RoboRioSim.getVInVoltage(), 0.02);
  }
}
