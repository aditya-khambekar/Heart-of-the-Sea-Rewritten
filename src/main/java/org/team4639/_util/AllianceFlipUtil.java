package org.team4639._util;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Rotation3d;
import org.team4639.constants.FieldConstants;

public class AllianceFlipUtil {
  public static Pose2d flipIfRedAlliance(Pose2d pose) {
    return DriverStationUtil.isBlueAlliance()
        ? pose
        : new Pose2d(
            FieldConstants.fieldLength - pose.getX(),
            FieldConstants.fieldWidth - pose.getY(),
            pose.getRotation().plus(Rotation2d.k180deg));
  }

  public static Pose3d flipIfRedAlliance(Pose3d pose) {
    var z = pose.getZ();
    Pose2d flippedPose2d = flipIfRedAlliance(pose.toPose2d());
    return new Pose3d(
        flippedPose2d.getX(), flippedPose2d.getY(), z, new Rotation3d(flippedPose2d.getRotation()));
  }
}
