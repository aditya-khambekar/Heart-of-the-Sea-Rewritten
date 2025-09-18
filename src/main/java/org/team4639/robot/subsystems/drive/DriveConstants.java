package org.team4639.robot.subsystems.drive;

import static edu.wpi.first.units.Units.MetersPerSecond;

import edu.wpi.first.units.measure.LinearVelocity;

public class DriveConstants {
  public static class Params {
    public static final double LINEAR_MULTIPLIER = 2.0;
  }
  // @aditya-khambekar,@JCCo-Pilot These both need to be tuned
  public static final Double driverStatorCurrentLimit = 90.0;
  public static final Double rotatorStatorCurrentLimit = 40.0;

  public static final LinearVelocity MAX_LINEAR_MPS = MetersPerSecond.of(8.0);
}
