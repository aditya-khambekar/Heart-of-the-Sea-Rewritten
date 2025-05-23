package org.team4639._util;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Transform2d;

public class PoseUtil {
  static double reefLeftDistance = 0.2;
  static double reefRightDistance = 0.18;

  public static Pose2d ReefRelativeRightOf(Pose2d pose) {
    return pose.transformBy(new Transform2d(0, -reefRightDistance, Rotation2d.kZero));
  }

  public static Pose2d ReefRelativeLeftOf(Pose2d pose) {
    return pose.transformBy(new Transform2d(0, reefLeftDistance, Rotation2d.kZero));
  }
}
