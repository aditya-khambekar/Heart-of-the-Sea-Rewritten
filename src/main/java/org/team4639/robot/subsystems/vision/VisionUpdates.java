package org.team4639.robot.subsystems.vision;

import edu.wpi.first.math.Matrix;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.numbers.N1;
import edu.wpi.first.math.numbers.N3;
import edu.wpi.first.wpilibj.Timer;
import java.util.HashSet;
import java.util.Objects;
import java.util.Set;

/** Utility class to update vision measurements across the entire program. */
public class VisionUpdates implements Vision.VisionConsumer {
  private static volatile VisionUpdates instance = new VisionUpdates();
  private Set<Vision.VisionConsumer> consumers = new HashSet<>();
  private double lastVisionUpdate = Double.NEGATIVE_INFINITY;

  public static synchronized VisionUpdates getInstance() {
    return instance = Objects.requireNonNullElseGet(instance, VisionUpdates::new);
  }

  @Override
  public void accept(
      int cameraIndex,
      Pose2d visionRobotPoseMeters,
      double timestampSeconds,
      Matrix<N3, N1> visionMeasurementStdDevs) {
    consumers.forEach(
        c ->
            c.accept(
                cameraIndex, visionRobotPoseMeters, timestampSeconds, visionMeasurementStdDevs));
    lastVisionUpdate = Timer.getFPGATimestamp();
  }

  public static void addConsumer(Vision.VisionConsumer consumer) {
    getInstance().consumers.add(consumer);
  }

  public boolean isVisionActive() {
    return Timer.getFPGATimestamp() - lastVisionUpdate <= 1.0;
  }
}
