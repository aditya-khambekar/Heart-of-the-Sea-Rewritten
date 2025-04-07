package org.team4639.subsystems.vision;

import edu.wpi.first.math.Matrix;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.numbers.N1;
import edu.wpi.first.math.numbers.N3;

import java.util.Objects;
import java.util.Set;

/**
 * Utility class to update vision measurements across the entire program.
 */
public class VisionUpdates implements Vision.VisionConsumer {
    private static volatile VisionUpdates instance = new VisionUpdates();
    private Set<Vision.VisionConsumer> consumers;

    public static synchronized VisionUpdates getInstance() {
        return instance = Objects.requireNonNullElseGet(instance, VisionUpdates::new);
    }

    @Override
    public void accept(Pose2d visionRobotPoseMeters, double timestampSeconds, Matrix<N3, N1> visionMeasurementStdDevs) {
        consumers.forEach(c -> c.accept(visionRobotPoseMeters, timestampSeconds, visionMeasurementStdDevs));
    }

    public static void addConsumer(Vision.VisionConsumer consumer) {
        getInstance().consumers.add(consumer);
    }
}
