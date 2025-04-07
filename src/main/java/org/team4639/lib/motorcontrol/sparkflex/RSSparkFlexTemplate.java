package org.team4639.lib.motorcontrol.sparkflex;

import com.revrobotics.spark.SparkLowLevel;

/** Stores templates for using the {@link RSSparkFlex} with different motors. */
public final class RSSparkFlexTemplate {
  public static RSSparkFlex NEOVortex(int ID) {
    return new RSSparkFlex(ID, SparkLowLevel.MotorType.kBrushless);
  }
}
