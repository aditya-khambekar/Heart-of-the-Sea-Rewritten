package org.team4639.robot.subsystems;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import java.util.HashMap;
import java.util.Map;
import org.team4639.robot.constants.FieldConstants;

/** Tracks reef. */
public class ReefTracker extends SubsystemBase {
  private Map<Pose2d, boolean[]> reefs = new HashMap<Pose2d, boolean[]>();
  private Pose2d currentReefPose = new Pose2d();

  public ReefTracker() {}

  public Command scoreL1() {
    return runOnce(() -> reefs.get(currentReefPose)[0] = true);
  }

  public Command scoreL2() {
    return runOnce(() -> reefs.get(currentReefPose)[1] = true);
  }

  public Command scoreL3() {
    return runOnce(() -> reefs.get(currentReefPose)[2] = true);
  }

  public Command scoreL4() {
    return runOnce(() -> reefs.get(currentReefPose)[3] = true);
  }

  public int getRecommendedLevel() {
    var reef = reefs.get(currentReefPose);
    if (!reef[3]) return 4;
    if (!reef[2]) return 3;
    if (!reef[1]) return 2;
    if (!reef[0]) return 1;
    return 0;
  }

  public void setCurrentReefPose(Pose2d pose) {
    currentReefPose = pose;
  }

  public Command setCurrentReefPoseCommand(Pose2d pose) {
    return runOnce(() -> setCurrentReefPose(pose));
  }

  public void resetReefTracker() {
    reefs.put(
        FieldConstants.TargetPositions.REEF_A.getPose(),
        new boolean[] {false, false, false, false});
    reefs.put(
        FieldConstants.TargetPositions.REEF_B.getPose(),
        new boolean[] {false, false, false, false});
    reefs.put(
        FieldConstants.TargetPositions.REEF_C.getPose(),
        new boolean[] {false, false, false, false});
    reefs.put(
        FieldConstants.TargetPositions.REEF_D.getPose(),
        new boolean[] {false, false, false, false});
    reefs.put(
        FieldConstants.TargetPositions.REEF_E.getPose(),
        new boolean[] {false, false, false, false});
    reefs.put(
        FieldConstants.TargetPositions.REEF_F.getPose(),
        new boolean[] {false, false, false, false});
    reefs.put(
        FieldConstants.TargetPositions.REEF_G.getPose(),
        new boolean[] {false, false, false, false});
    reefs.put(
        FieldConstants.TargetPositions.REEF_H.getPose(),
        new boolean[] {false, false, false, false});
    reefs.put(
        FieldConstants.TargetPositions.REEF_I.getPose(),
        new boolean[] {false, false, false, false});
    reefs.put(
        FieldConstants.TargetPositions.REEF_J.getPose(),
        new boolean[] {false, false, false, false});
    reefs.put(
        FieldConstants.TargetPositions.REEF_K.getPose(),
        new boolean[] {false, false, false, false});
    reefs.put(
        FieldConstants.TargetPositions.REEF_L.getPose(),
        new boolean[] {false, false, false, false});
  }
}
