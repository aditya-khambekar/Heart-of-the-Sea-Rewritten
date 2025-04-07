package org.team4639._lib.motorcontrol.sparkmax;

import com.revrobotics.spark.SparkLowLevel;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.math.system.plant.LinearSystemId;

/** Stores templates for using the {@link RSSparkMax} with different motors. */
public final class RSSparkMaxTemplate {
  public static RSSparkMax NEOVortex(int ID) {
    return new RSSparkMax(ID, SparkLowLevel.MotorType.kBrushless);
  }

  public static RSSparkMax NEO550(int ID) {
    var max = new RSSparkMax(ID, SparkLowLevel.MotorType.kBrushless);
    max.setSimGearbox(DCMotor.getNeo550(1));
    max.setSimPlant(
        LinearSystemId.createDCMotorSystem(max.gearbox, 0.4445 * Math.pow(0.025, 2), 1.0));
    return max;
  }

  public static RSSparkMax NEO(int ID) {
    var max = new RSSparkMax(ID, SparkLowLevel.MotorType.kBrushless);
    max.setSimGearbox(DCMotor.getNEO(1));
    max.setSimPlant(LinearSystemId.createDCMotorSystem(max.gearbox, 6.660E-05, 1.0));
    return max;
  }

  public static RSSparkMax Vex775(int ID) {
    var max = new RSSparkMax(ID, SparkLowLevel.MotorType.kBrushed);
    max.setSimGearbox(DCMotor.getVex775Pro(1));
    max.setSimPlant(LinearSystemId.createDCMotorSystem(max.gearbox, 6.660E-05, 1.0));
    return max;
  }
}
