package org.team4639.subsystems.vision;

import edu.wpi.first.math.Matrix;
import edu.wpi.first.math.filter.LinearFilter;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.numbers.N1;
import edu.wpi.first.math.numbers.N3;
import java.util.stream.IntStream;

public class RawVisionPoses {
  public LinearFilterPoseEstimator frontCameras = new LinearFilterPoseEstimator(1, 2);

  public static class LinearFilterPoseEstimator {
    private final LinearFilter xFilter = LinearFilter.singlePoleIIR(0.1, 0.02);
    private final LinearFilter yFilter = LinearFilter.singlePoleIIR(0.1, 0.02);
    private final LinearFilter omegaFilter = LinearFilter.singlePoleIIR(0.1, 0.02);
    int[] acceptedIndices;

    private Pose2d poseEstimate = new Pose2d();

    public void accept(
        int CameraIndex,
        Pose2d visionRobotPoseMeters,
        double timestampSeconds,
        Matrix<N3, N1> visionMeasurementStdDevs) {
      if (IntStream.of(acceptedIndices).anyMatch(idx -> idx == CameraIndex)) {
        poseEstimate =
            new Pose2d(
                xFilter.calculate(visionRobotPoseMeters.getX()),
                yFilter.calculate(visionRobotPoseMeters.getY()),
                Rotation2d.fromRadians(
                    omegaFilter.calculate(visionRobotPoseMeters.getRotation().getRadians())));
      }
    }

    public Pose2d getPoseEstimate() {
      return poseEstimate;
    }

    public LinearFilterPoseEstimator(int... acceptedIndices) {
      this.acceptedIndices = acceptedIndices;
    }
  }
}
