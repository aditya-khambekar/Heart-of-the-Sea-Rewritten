package org.team4639.subsystems.scoring;

import au.grapplerobotics.ConfigurationFailedException;
import au.grapplerobotics.LaserCan;
import au.grapplerobotics.interfaces.LaserCanInterface.Measurement;
import au.grapplerobotics.interfaces.LaserCanInterface.RangingMode;
import com.revrobotics.spark.SparkMax;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

public class ScoringIOHardware implements ScoringIO {
  SparkMax spark;
  LaserCan laserCan;

  public ScoringIOHardware(SparkMax spark, LaserCan laserCan) {
    this.spark = spark;
    this.laserCan = laserCan;

    try {
      laserCan.setRangingMode(RangingMode.SHORT);
    } catch (ConfigurationFailedException e) {
      // TODO Auto-generated catch block
      e.printStackTrace();
    }
  }

  public void updateInputs(ScoringIOInputs inputs) {
    if (spark != null) {
      inputs.currentAmps = spark.getOutputCurrent();
    }
    if (laserCan != null) {
      Measurement measurement = laserCan.getMeasurement();
      inputs.hasCoral =
          measurement != null
              && measurement.distance_mm < 20
              && measurement.status == LaserCan.LASERCAN_STATUS_VALID_MEASUREMENT;
    }
    SmartDashboard.putNumber("scoring/motor temp", spark.getMotorTemperature());
  }

  public void runSparkMax(double speed) {
    // spark.set(speed);
  }
}
