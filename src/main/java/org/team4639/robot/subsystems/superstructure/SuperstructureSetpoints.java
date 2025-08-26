package org.team4639.robot.subsystems.superstructure;

import static edu.wpi.first.units.Units.*;

import edu.wpi.first.math.geometry.Rotation2d;

public class SuperstructureSetpoints {
  public static final SuperstructureState IDLE =
      new SuperstructureState(
          Value.of(0.04), Rotation2d.fromDegrees(150), RotationsPerSecond.of(0.0));

  public static final SuperstructureState CORAL_STOW =
      new SuperstructureState(
          Value.of(0.04), Rotation2d.fromDegrees(150), RotationsPerSecond.of(0.0));

  public static final SuperstructureState HP =
      new SuperstructureState(
          Value.of(0.048), Rotation2d.fromDegrees(77.6), RotationsPerSecond.of(5));

  public static final SuperstructureState PROCESSOR =
      new SuperstructureState(
          Value.of(0.15), Rotation2d.fromDegrees(100), RotationsPerSecond.of(8));

  public static final SuperstructureState L4_PREP =
      new SuperstructureState(
          Value.of(0.40), Rotation2d.fromDegrees(110), RotationsPerSecond.of(0));

  public static final SuperstructureState L3_PREP =
      new SuperstructureState(
          Value.of(0.40), Rotation2d.fromDegrees(140), RotationsPerSecond.of(0));

  public static final SuperstructureState L2_PREP =
      new SuperstructureState(
          Value.of(0.2893), Rotation2d.fromDegrees(220), RotationsPerSecond.of(0));

  public static final SuperstructureState L1_PREP =
      new SuperstructureState(
          Value.of(0.15), Rotation2d.fromDegrees(220), RotationsPerSecond.of(0));

  public static final SuperstructureState AUTO_ELEVATOR_L4_PREP = L4_PREP;

  public static final SuperstructureState L1 =
      new SuperstructureState(
          Value.of(0.15), Rotation2d.fromDegrees(220), RotationsPerSecond.of(-4));

  public static final SuperstructureState L2 =
      new SuperstructureState(
          Value.of(0.2893), Rotation2d.fromDegrees(220), RotationsPerSecond.of(-18));

  public static final SuperstructureState L3 =
      new SuperstructureState(
          Value.of(0.496), Rotation2d.fromDegrees(220), RotationsPerSecond.of(-18));

  public static final SuperstructureState L4 =
      new SuperstructureState(
          Value.of(0.85635), Rotation2d.fromDegrees(230), RotationsPerSecond.of(-18));

  public static final SuperstructureState L2_ALGAE =
      new SuperstructureState(
          Value.of(0.32), Rotation2d.fromDegrees(135), RotationsPerSecond.of(10));

  public static final SuperstructureState L3_ALGAE =
      new SuperstructureState(
          Value.of(0.8), Rotation2d.fromDegrees(135), RotationsPerSecond.of(10));

  public static final SuperstructureState ALGAE_STOW =
      new SuperstructureState(
          Value.of(0.32), Rotation2d.fromDegrees(135), RotationsPerSecond.of(0));

  public static final SuperstructureState BARGE =
      new SuperstructureState(
          Value.of(0.95), Rotation2d.fromDegrees(150), RotationsPerSecond.of(3));

  public static final SuperstructureState BARGE_NO_OUTTAKE =
      new SuperstructureState(
          Value.of(0.95), Rotation2d.fromDegrees(150), RotationsPerSecond.of(3));

  public static final SuperstructureState GROUND_INTAKE =
      new SuperstructureState(
          Value.of(0.0), Rotation2d.fromDegrees(135), RotationsPerSecond.of(-9));

  public static final SuperstructureState HOMING =
      new SuperstructureState(Value.of(0.0), Rotation2d.fromDegrees(135), RotationsPerSecond.of(0));

  public static final SuperstructureState HP_LOWER =
      new SuperstructureState(
          Value.of(0.048), Rotation2d.fromDegrees(58), RotationsPerSecond.of(13));

  public static final SuperstructureState HOMING_READY =
      new SuperstructureState(
          Value.of(0.1), Rotation2d.fromDegrees(135), RotationsPerSecond.zero());

  public static final SuperstructureState REJECT_CORAL =
      new SuperstructureState(Value.of(0), Rotation2d.fromDegrees(150), RotationsPerSecond.of(-5));

  public static final SuperstructureState REJECT_ALGAE =
      new SuperstructureState(Value.of(0), Rotation2d.fromDegrees(135), RotationsPerSecond.of(5));
}
