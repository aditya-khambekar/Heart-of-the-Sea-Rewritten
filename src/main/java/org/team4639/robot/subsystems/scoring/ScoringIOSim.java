package org.team4639.robot.subsystems.scoring;

import au.grapplerobotics.ConfigurationFailedException;
import au.grapplerobotics.LaserCan;
import au.grapplerobotics.interfaces.LaserCanInterface.Measurement;
import au.grapplerobotics.interfaces.LaserCanInterface.RangingMode;
import au.grapplerobotics.simulation.MockLaserCan;
import com.revrobotics.spark.SparkMax;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

public class ScoringIOSim implements ScoringIO {
  SparkMax spark;
  MockLaserCan laserCan;

  public ScoringIOSim(SparkMax spark, MockLaserCan laserCan) {
    this.spark = spark;
    this.laserCan = laserCan;

    try {
      laserCan.setRangingMode(RangingMode.SHORT);
    } catch (ConfigurationFailedException e) {
      // TODO Auto-generated catch block
      e.printStackTrace();
    }

    SmartDashboard.putBoolean("Sim Has Coral", false);
  }

  public void updateInputs(ScoringIOInputs inputs) {
    if (spark != null) {
      inputs.currentAmps = spark.getOutputCurrent();
    }
    if (laserCan != null) {
      if (SmartDashboard.getBoolean("Sim Has Coral", false)) {
        laserCan.setMeasurementPartialSim(LaserCan.LASERCAN_STATUS_VALID_MEASUREMENT, 10, 0);
      } else {
        laserCan.setMeasurementPartialSim(LaserCan.LASERCAN_STATUS_VALID_MEASUREMENT, 100, 0);
      }

      Measurement measurement = laserCan.getMeasurement();
      inputs.hasCoral =
          measurement != null
              && measurement.distance_mm < 20
              && measurement.status == LaserCan.LASERCAN_STATUS_VALID_MEASUREMENT;
    }
  }

  public void runSparkMax(double speed) {
    spark.set(speed);
  }
}
