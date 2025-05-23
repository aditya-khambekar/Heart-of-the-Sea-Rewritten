// Copyright (c) 2025 FRC 6328
// http://github.com/Mechanical-Advantage
//
// Use of this source code is governed by an MIT-style
// license that can be found in the LICENSE file at
// the root directory of this project.

package org.team4639.constants;

import edu.wpi.first.math.geometry.*;
import edu.wpi.first.math.util.Units;
import java.util.HashMap;
import java.util.Map;
import org.team4639._util.PoseUtil;

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
    initAlgaeLocations();
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

  public static class Reef {
    public static final Translation2d center =
        new Translation2d(Units.inchesToMeters(176.746), Units.inchesToMeters(158.501));
    public static final double faceToZoneLine =
        Units.inchesToMeters(12); // Side of the reef to the inside of the reef zone line

    public static final Pose2d[] centerFaces =
        new Pose2d[6]; // Starting facing the driver station in clockwise order

    public static void init() {
      // Initialize faces
      centerFaces[0] =
          new Pose2d(
              Units.inchesToMeters(144.003),
              Units.inchesToMeters(158.500),
              Rotation2d.fromDegrees(180));
      centerFaces[1] =
          new Pose2d(
              Units.inchesToMeters(160.373),
              Units.inchesToMeters(186.857),
              Rotation2d.fromDegrees(120));
      centerFaces[2] =
          new Pose2d(
              Units.inchesToMeters(193.116),
              Units.inchesToMeters(186.858),
              Rotation2d.fromDegrees(60));
      centerFaces[3] =
          new Pose2d(
              Units.inchesToMeters(209.489),
              Units.inchesToMeters(158.502),
              Rotation2d.fromDegrees(0));
      centerFaces[4] =
          new Pose2d(
              Units.inchesToMeters(193.118),
              Units.inchesToMeters(130.145),
              Rotation2d.fromDegrees(-60));
      centerFaces[5] =
          new Pose2d(
              Units.inchesToMeters(160.375),
              Units.inchesToMeters(130.144),
              Rotation2d.fromDegrees(-120));
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

  public static Map<Pose2d, Integer> ReefCenterPoseToAlgaeLocation = new HashMap<>();

  public static void initAlgaeLocations() {
    ReefCenterPoseToAlgaeLocation.put(TargetPositions.REEF_AB.Pose, 0b1);
    ReefCenterPoseToAlgaeLocation.put(TargetPositions.REEF_CD.Pose, 0b0);
    ReefCenterPoseToAlgaeLocation.put(TargetPositions.REEF_EF.Pose, 0b1);
    ReefCenterPoseToAlgaeLocation.put(TargetPositions.REEF_GH.Pose, 0b0);
    ReefCenterPoseToAlgaeLocation.put(TargetPositions.REEF_IJ.Pose, 0b1);
    ReefCenterPoseToAlgaeLocation.put(TargetPositions.REEF_KL.Pose, 0b0);
  }
}
