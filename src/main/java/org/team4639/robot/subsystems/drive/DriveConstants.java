package org.team4639.robot.subsystems.drive;

import static edu.wpi.first.units.Units.MetersPerSecond;

import com.pathplanner.lib.config.PIDConstants;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.units.measure.LinearVelocity;

public final class DriveConstants {
  public static class Params {
    public static final double LINEAR_MULTIPLIER = 2.0;
  }
  // @aditya-khambekar,@JCCo-Pilot These both need to be tuned
  public static final Double driverStatorCurrentLimit = 90.0;
  public static final Double rotatorStatorCurrentLimit = 40.0;

  public static final LinearVelocity MAX_LINEAR_MPS = MetersPerSecond.of(5.0);

  public static final PIDConstants ALIGN_TRANSLATION_PID_CONSTANTS = new PIDConstants(4, 0, 0.15);
  public static final TrapezoidProfile.Constraints ALIGN_TRANSLATION_CONSTRAINTS =
      new TrapezoidProfile.Constraints(3, 5);
  public static final PIDConstants ALIGN_ROTATION_PID_CONSTANTS = new PIDConstants(8, 0, 0);
}
