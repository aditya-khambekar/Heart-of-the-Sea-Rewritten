package org.team4639.robot.subsystems.drive;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import java.util.HashSet;
import java.util.Objects;
import java.util.Optional;
import java.util.Set;

public class BaseDriveConsumer implements DriveBase.DriveConsumer {
  private static volatile BaseDriveConsumer instance = new BaseDriveConsumer();
  private Set<DriveBase.DriveConsumer> consumers = new HashSet<>();
  private double lastVisionUpdate = Double.NEGATIVE_INFINITY;

  public static synchronized BaseDriveConsumer getInstance() {
    return instance = Objects.requireNonNullElseGet(instance, BaseDriveConsumer::new);
  }

  public static void addConsumer(DriveBase.DriveConsumer consumer) {
    getInstance().consumers.add(consumer);
  }

  @Override
  public void accept(
      Pose2d estimatedPose,
      SwerveModuleState[] moduleStates,
      SwerveModulePosition[] wheelPositions,
      Optional<Rotation2d> gyroAngle,
      double timestamp) {
    consumers.forEach(
        c -> c.accept(estimatedPose, moduleStates, wheelPositions, gyroAngle, timestamp));
  }
}
