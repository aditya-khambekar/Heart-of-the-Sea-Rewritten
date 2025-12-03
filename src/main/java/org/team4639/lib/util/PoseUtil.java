package org.team4639.lib.util;

import static edu.wpi.first.units.Units.Meter;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Transform2d;
import edu.wpi.first.units.measure.Distance;

public class PoseUtil {
  static double reefLeftDistance = 0.2;
  static double reefRightDistance = 0.17;

  public static Pose2d ReefRelativeRightOf(Pose2d pose) {
    return pose.transformBy(new Transform2d(0, -reefRightDistance, Rotation2d.kZero));
  }

  public static Pose2d ReefRelativeLeftOf(Pose2d pose) {
    return pose.transformBy(new Transform2d(0, reefLeftDistance, Rotation2d.kZero));
  }

  public static Distance getDistance(Pose2d one, Pose2d other) {
    return Meter.of(one.getTranslation().getDistance(other.getTranslation()));
  }
}
