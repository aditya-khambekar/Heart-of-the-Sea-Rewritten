package org.team4639.lib.util;

import edu.wpi.first.math.geometry.Rotation2d;
import java.util.Arrays;

public class RotationUtil {
  public static Rotation2d nearest(Rotation2d target, Rotation2d... rotations) {
    var x = rotations[0];

    for (var y : rotations) {
      if (Math.abs(target.getDegrees() - y.getDegrees())
          < Math.abs(target.getDegrees() - x.getDegrees())) x = y;
    }

    return x;
  }

  public static Rotation2d max(Rotation2d one, Rotation2d other) {
    return one.getDegrees() > other.getDegrees() ? one : other;
  }

  public static Rotation2d min(Rotation2d one, Rotation2d other) {
    return one.getDegrees() < other.getDegrees() ? one : other;
  }

  /**
   * @param one
   * @param other
   * @return one minus other
   */
  public static double compare(Rotation2d one, Rotation2d other) {
    return one.getDegrees() - other.getDegrees();
  }

  public static Rotation2d average(Rotation2d... args) {
    return Rotation2d.fromDegrees(
        Arrays.stream(args).mapToDouble(x -> x.getDegrees()).average().orElse(-1));
  }

  public static boolean boundedBy(Rotation2d target, Rotation2d fence1, Rotation2d fence2) {
    return compare(target, max(fence1, fence2)) <= 0 && compare(target, min(fence1, fence2)) >= 0;
  }
}
