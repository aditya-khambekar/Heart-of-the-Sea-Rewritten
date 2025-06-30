// Copyright (c) 2025 FRC 6328
// http://github.com/Mechanical-Advantage
//
// Use of this source code is governed by an MIT-style
// license that can be found in the LICENSE file at
// the root directory of this project.

package org.team4639.robot.constants;

import com.fasterxml.jackson.core.JsonProcessingException;
import com.fasterxml.jackson.databind.ObjectMapper;
import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.math.geometry.*;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj2.command.Command;
import java.io.IOException;
import java.nio.file.Path;
import java.util.*;
import org.team4639.lib.util.PoseUtil;
import org.team4639.robot.commands.SuperstructureCommands;

/**
 * Contains various field dimensions and useful reference points. All units are in meters and poses
 * have a blue alliance origin.
 */
public class FieldConstants {
  public static final double fieldLength = Units.inchesToMeters(690.876);
  public static final double fieldWidth = Units.inchesToMeters(317);
  public static final double startingLineX =
      Units.inchesToMeters(299.438); // Measured from the inside of starting line

  public static void init() {
    Reef.init();
  }

  public enum AprilTagLayoutType {
    OFFICIAL("2025-official"),
    NO_BARGE("2025-no-barge"),
    BLUE_REEF("2025-blue-reef"),
    RED_REEF("2025-red-reef"),
    NONE("2025-none");

    AprilTagLayoutType(String name) {
      try {
        layout =
            new AprilTagFieldLayout(
                Path.of("src", "main", "deploy", "apriltags", "andymark", "2025-official.json"));
      } catch (IOException e) {
        throw new RuntimeException(e);
      }

      try {
        layoutString = new ObjectMapper().writeValueAsString(layout);
      } catch (JsonProcessingException e) {
        throw new RuntimeException(
            "Failed to serialize AprilTag layout JSON " + toString() + "for Northstar");
      }
    }

    private final AprilTagFieldLayout layout;
    private final String layoutString;

    public AprilTagFieldLayout getLayout() {
      return layout;
    }
  }

  public static class Processor {
    public static final Pose2d centerFace =
        new Pose2d(Units.inchesToMeters(235.726), 0, Rotation2d.fromDegrees(90));
  }

  public static class Barge {
    public static final Translation2d farCage =
        new Translation2d(Units.inchesToMeters(345.428), Units.inchesToMeters(286.779));
    public static final Translation2d middleCage =
        new Translation2d(Units.inchesToMeters(345.428), Units.inchesToMeters(242.855));
    public static final Translation2d closeCage =
        new Translation2d(Units.inchesToMeters(345.428), Units.inchesToMeters(199.947));

    // Measured from floor to bottom of cage
    public static final double deepHeight = Units.inchesToMeters(3.125);
    public static final double shallowHeight = Units.inchesToMeters(30.125);
  }

  public static class CoralStation {
    public static final Pose2d leftCenterFace =
        new Pose2d(
            Units.inchesToMeters(33.526),
            Units.inchesToMeters(291.176),
            Rotation2d.fromDegrees(90 - 144.011));
    public static final Pose2d rightCenterFace =
        new Pose2d(
            Units.inchesToMeters(33.526),
            Units.inchesToMeters(25.824),
            Rotation2d.fromDegrees(144.011 - 90));
  }

  public enum ReefLevel {
    L1(0, Units.inchesToMeters(25.0), 0),
    L2(1, Units.inchesToMeters(31.875 - Math.cos(Math.toRadians(35.0)) * 0.625), -35),
    L3(2, Units.inchesToMeters(47.625 - Math.cos(Math.toRadians(35.0)) * 0.625), -35),
    L4(3, Units.inchesToMeters(72), -90);

    ReefLevel(int levelNumber, double height, double pitch) {
      this.levelNumber = levelNumber;
      this.height = height;
      this.pitch = pitch; // Degrees
    }

    public static ReefLevel fromLevel(int level) {
      return Arrays.stream(values())
          .filter(height -> height.ordinal() == level)
          .findFirst()
          .orElse(L4);
    }

    public final int levelNumber;
    public final double height;
    public final double pitch;
  }

  public static final double aprilTagWidth = Units.inchesToMeters(6.50);
  public static final int aprilTagCount = 22;
  public static final AprilTagLayoutType defaultAprilTagType = AprilTagLayoutType.NO_BARGE;

  public static class Reef {
    public static final double faceLength = Units.inchesToMeters(36.792600);
    public static final Translation2d center =
        new Translation2d(Units.inchesToMeters(176.746), fieldWidth / 2.0);
    public static final double faceToZoneLine =
        Units.inchesToMeters(12); // Side of the reef to the inside of the reef zone line

    public static final Pose2d[] centerFaces =
        new Pose2d[6]; // Starting facing the driver station in clockwise order
    public static final List<Map<ReefLevel, Pose3d>> branchPositions =
        new ArrayList<>(); // Starting at the right branch facing the driver station in clockwise
    public static final List<Map<ReefLevel, Pose2d>> branchPositions2d = new ArrayList<>();

    public static void init() {
      // Initialize faces
      var aprilTagLayout = AprilTagLayoutType.OFFICIAL.getLayout();
      centerFaces[0] = aprilTagLayout.getTagPose(18).get().toPose2d();
      centerFaces[1] = aprilTagLayout.getTagPose(19).get().toPose2d();
      centerFaces[2] = aprilTagLayout.getTagPose(20).get().toPose2d();
      centerFaces[3] = aprilTagLayout.getTagPose(21).get().toPose2d();
      centerFaces[4] = aprilTagLayout.getTagPose(22).get().toPose2d();
      centerFaces[5] = aprilTagLayout.getTagPose(17).get().toPose2d();

      // Initialize branch positions
      for (int face = 0; face < 6; face++) {
        Map<ReefLevel, Pose3d> fillRight = new HashMap<>();
        Map<ReefLevel, Pose3d> fillLeft = new HashMap<>();
        Map<ReefLevel, Pose2d> fillRight2d = new HashMap<>();
        Map<ReefLevel, Pose2d> fillLeft2d = new HashMap<>();
        for (var level : ReefLevel.values()) {
          Pose2d poseDirection = new Pose2d(center, Rotation2d.fromDegrees(180 - (60 * face)));
          double adjustX = Units.inchesToMeters(30.738);
          double adjustY = Units.inchesToMeters(6.469);

          var rightBranchPose =
              new Pose3d(
                  new Translation3d(
                      poseDirection
                          .transformBy(new Transform2d(adjustX, adjustY, Rotation2d.kZero))
                          .getX(),
                      poseDirection
                          .transformBy(new Transform2d(adjustX, adjustY, Rotation2d.kZero))
                          .getY(),
                      level.height),
                  new Rotation3d(
                      0,
                      Units.degreesToRadians(level.pitch),
                      poseDirection.getRotation().getRadians()));
          var leftBranchPose =
              new Pose3d(
                  new Translation3d(
                      poseDirection
                          .transformBy(new Transform2d(adjustX, -adjustY, Rotation2d.kZero))
                          .getX(),
                      poseDirection
                          .transformBy(new Transform2d(adjustX, -adjustY, Rotation2d.kZero))
                          .getY(),
                      level.height),
                  new Rotation3d(
                      0,
                      Units.degreesToRadians(level.pitch),
                      poseDirection.getRotation().getRadians()));

          fillRight.put(level, rightBranchPose);
          fillLeft.put(level, leftBranchPose);
          fillRight2d.put(level, rightBranchPose.toPose2d());
          fillLeft2d.put(level, leftBranchPose.toPose2d());
        }
        branchPositions.add(fillRight);
        branchPositions.add(fillLeft);
        branchPositions2d.add(fillRight2d);
        branchPositions2d.add(fillLeft2d);
      }
    }
  }

  public static class StagingPositions {
    // Measured from the center of the ice cream
    public static final Pose2d leftIceCream =
        new Pose2d(Units.inchesToMeters(48), Units.inchesToMeters(230.5), new Rotation2d());
    public static final Pose2d middleIceCream =
        new Pose2d(Units.inchesToMeters(48), Units.inchesToMeters(158.5), new Rotation2d());
    public static final Pose2d rightIceCream =
        new Pose2d(Units.inchesToMeters(48), Units.inchesToMeters(86.5), new Rotation2d());
  }

  public enum ReefHeight {
    L4(Units.inchesToMeters(72), -90),
    L3(Units.inchesToMeters(47.625), -35),
    L2(Units.inchesToMeters(31.875), -35),
    L1(Units.inchesToMeters(18), 0);

    ReefHeight(double height, double pitch) {
      this.height = height;
      this.pitch = pitch; // in degrees
    }

    public final double height;
    public final double pitch;
  }

  // change this to tune how far the align tries to go from the reef face
  static Transform2d fromReef = new Transform2d(Units.inchesToMeters(29.25), 0, Rotation2d.k180deg);
  static Transform2d fromProcessor =
      new Transform2d(Units.inchesToMeters(25), 0, Rotation2d.k180deg);
  // change this to tune how far the align tries to go from the intake station
  static Transform2d fromCoralStation =
      new Transform2d(Units.inchesToMeters(16), 0, Rotation2d.kZero);
  static Transform2d fromBarge = new Transform2d(Units.inchesToMeters(-39), 0, Rotation2d.kZero);

  public static enum TargetPositions {
    REEF_AB(FieldConstants.Reef.centerFaces[0].transformBy((FieldConstants.fromReef))),
    REEF_CD(FieldConstants.Reef.centerFaces[5].transformBy((FieldConstants.fromReef))),
    REEF_EF(FieldConstants.Reef.centerFaces[4].transformBy((FieldConstants.fromReef))),
    REEF_GH(FieldConstants.Reef.centerFaces[3].transformBy((FieldConstants.fromReef))),
    REEF_IJ(FieldConstants.Reef.centerFaces[2].transformBy((FieldConstants.fromReef))),
    REEF_KL(FieldConstants.Reef.centerFaces[1].transformBy((FieldConstants.fromReef))),

    REEF_A(PoseUtil.ReefRelativeLeftOf(REEF_AB.getPose())),
    REEF_B(PoseUtil.ReefRelativeRightOf(REEF_AB.getPose())),
    REEF_C(PoseUtil.ReefRelativeLeftOf(REEF_CD.getPose())),
    REEF_D(PoseUtil.ReefRelativeRightOf(REEF_CD.getPose())),
    REEF_E(PoseUtil.ReefRelativeLeftOf(REEF_EF.getPose())),
    REEF_F(PoseUtil.ReefRelativeRightOf(REEF_EF.getPose())),
    REEF_G(PoseUtil.ReefRelativeLeftOf(REEF_GH.getPose())),
    REEF_H(PoseUtil.ReefRelativeRightOf(REEF_GH.getPose())),
    REEF_I(PoseUtil.ReefRelativeLeftOf(REEF_IJ.getPose())),
    REEF_J(PoseUtil.ReefRelativeRightOf(REEF_IJ.getPose())),
    REEF_K(PoseUtil.ReefRelativeLeftOf(REEF_KL.getPose())),
    REEF_L(PoseUtil.ReefRelativeRightOf(REEF_KL.getPose())),

    PROCESSOR(FieldConstants.Processor.centerFace.transformBy(FieldConstants.fromProcessor)),

    CORALSTATION_LEFT(
        FieldConstants.CoralStation.leftCenterFace.transformBy(FieldConstants.fromCoralStation)),
    CORALSTATION_RIGHT(
        FieldConstants.CoralStation.rightCenterFace.transformBy(FieldConstants.fromCoralStation)),

    BARGE_FARCAGE(
        new Pose2d(FieldConstants.Barge.farCage, Rotation2d.kZero).transformBy(fromBarge)),
    BARGE_MIDDLECAGE(
        new Pose2d(FieldConstants.Barge.middleCage, Rotation2d.kZero).transformBy(fromBarge)),
    BARGE_CLOSECAGE(
        new Pose2d(FieldConstants.Barge.closeCage, Rotation2d.kZero).transformBy(fromBarge));

    TargetPositions(Pose2d pose, Pose2d leftPose, Pose2d rightPose) {
      this.leftPose = leftPose;
      this.rightPose = rightPose;
      this.Pose = pose;
    }

    TargetPositions(Pose2d pose) {
      this(pose, PoseUtil.ReefRelativeLeftOf(pose), PoseUtil.ReefRelativeRightOf(pose));
    }

    public Pose2d getPose() {
      return Pose;
    }

    private final Pose2d Pose;
    public final Pose2d leftPose, rightPose;
  }

  public static Map<Pose2d, Command> ReefCenterPoseToAlgaeLocation = new HashMap<>();

  public static void initAlgaeLocations() {
    ReefCenterPoseToAlgaeLocation.put(
        TargetPositions.REEF_AB.Pose, SuperstructureCommands.L3_ALGAE);
    ReefCenterPoseToAlgaeLocation.put(
        TargetPositions.REEF_CD.Pose, SuperstructureCommands.L2_ALGAE);
    ReefCenterPoseToAlgaeLocation.put(
        TargetPositions.REEF_EF.Pose, SuperstructureCommands.L3_ALGAE);
    ReefCenterPoseToAlgaeLocation.put(
        TargetPositions.REEF_GH.Pose, SuperstructureCommands.L2_ALGAE);
    ReefCenterPoseToAlgaeLocation.put(
        TargetPositions.REEF_IJ.Pose, SuperstructureCommands.L3_ALGAE);
    ReefCenterPoseToAlgaeLocation.put(
        TargetPositions.REEF_KL.Pose, SuperstructureCommands.L2_ALGAE);
  }

  public static Map<Pose2d, Command> ReefCenterPoseToAlgaeLocation() {
    var x = new HashMap<Pose2d, Command>();
    x.put(TargetPositions.REEF_AB.Pose, SuperstructureCommands.l3Algae());
    x.put(TargetPositions.REEF_CD.Pose, SuperstructureCommands.l2Algae());
    x.put(TargetPositions.REEF_EF.Pose, SuperstructureCommands.l3Algae());
    x.put(TargetPositions.REEF_GH.Pose, SuperstructureCommands.l2Algae());
    x.put(TargetPositions.REEF_IJ.Pose, SuperstructureCommands.l3Algae());
    x.put(TargetPositions.REEF_KL.Pose, SuperstructureCommands.l2Algae());

    return x;
  }

  public static Pose2d getClosestBranchPosition(Pose2d currentPose) {
    List<Pose2d> branches = new ArrayList<>();
    for (Map<ReefLevel, Pose2d> x : Reef.branchPositions2d) {
      branches.addAll(x.values());
    }

    return currentPose.nearest(branches);
  }

  public static Rotation2d getRotationToClosestBranchPosition(Pose2d currentPose) {
    var closestBranchPosition = getClosestBranchPosition(currentPose);
    return Rotation2d.fromRadians(
        Math.atan2(
            closestBranchPosition.getY() - currentPose.getY(),
            closestBranchPosition.getX() - currentPose.getX()));
  }
}
