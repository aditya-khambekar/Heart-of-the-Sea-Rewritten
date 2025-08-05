package org.team4639.robot.subsystems.superstructure;

import static edu.wpi.first.units.Units.*;

import edu.wpi.first.math.geometry.Rotation2d;

public class SuperstructureSetpoints {
  // IDLE State
  public static final SuperstructureState IDLE =
      new SuperstructureState(
          Value.of(0.04), // IDLE_Proportion
          Rotation2d.fromDegrees(150), // Wrist_IDLE_Proportion: 30 - 230 * 0.414 = -65.22 degrees
          RotationsPerSecond.of(0.0) // Intake_Idle_Speed: 0.0 * 20 = 0.0 RPS
          );

  public static final SuperstructureState CORAL_STOW =
      new SuperstructureState(
          Value.of(0.04), // IDLE_Proportion
          Rotation2d.fromDegrees(150), // Wrist_IDLE_Proportion: 30 - 230 * 0.414 = -65.22 degrees
          RotationsPerSecond.of(0.0) // Intake_Idle_Speed: 0.0 * 20 = 0.0 RPS
          );

  // HP State
  public static final SuperstructureState HP =
      new SuperstructureState(
          Value.of(0.048), // HP_Proportion
          Rotation2d.fromDegrees(55), // Wrist_HP_Proportion: 30 - 230 * 0.238 = -24.74 degrees
          RotationsPerSecond.of(5) // Intake_HP_Speed: 0.5 * 20 = 10.0 RPS
          );

  // Processor State
  public static final SuperstructureState PROCESSOR =
      new SuperstructureState(
          Value.of(0.15), // Processor_Proportion: ProportionToPosition.convertBackwards(2.620) -
          // assuming this equals ~0.655
          Rotation2d.fromDegrees(
              100), // Wrist_Processor_Proportion: 30 - 230 * 0.709 = -133.07 degrees
          RotationsPerSecond.of(8) // Intake_Processor_Speed: 0.5 * 20 = 10.0 RPS
          );

  public static final SuperstructureState ELEVATOR_READY =
      new SuperstructureState(
          Value.of(0.40), Rotation2d.fromDegrees(110), RotationsPerSecond.of(0));

  public static final SuperstructureState AUTO_ELEVATOR_L4_READY =
      new SuperstructureState(Value.of(0.7), Rotation2d.fromDegrees(230), RotationsPerSecond.of(0));

  // L1 State
  public static final SuperstructureState L1 =
      new SuperstructureState(
          Value.of(0.15), // L1_Proportion
          Rotation2d.fromDegrees(215), // Wrist_L1_Proportion: 30 - 230 * 1.0 = -200.0 degrees
          RotationsPerSecond.of(-4) // Intake_L1_Speed: 0.2 * 20 = 4.0 RPS
          );

  // L2 State
  public static final SuperstructureState L2 =
      new SuperstructureState(
          Value.of(0.2893), // L2_Proportion
          Rotation2d.fromDegrees(215), // Wrist_L2_Proportion: 30 - 230 * 0.95 = -188.5 degrees
          RotationsPerSecond.of(-9) // Intake_L2_Speed: -0.875 * 20 = -17.5 RPS
          );

  // L3 State
  public static final SuperstructureState L3 =
      new SuperstructureState(
          Value.of(0.496), // L3_Proportion
          Rotation2d.fromDegrees(215), // Wrist_L3_Proportion: 30 - 230 * 0.95 = -188.5 degrees
          RotationsPerSecond.of(-9) // Intake_L3_Speed: -0.875 * 20 = -17.5 RPS
          );

  // L4 State
  public static final SuperstructureState L4 =
      new SuperstructureState(
          Value.of(0.85635), // L4_Proportion
          Rotation2d.fromDegrees(230), // Wrist_L4_Proportion: 30 - 230 * 1.0 = -200.0 degrees
          RotationsPerSecond.of(-18) // Intake_L4_Speed: -0.875 * 20 = -17.5 RPS
          );

  // L2_ALGAE State
  public static final SuperstructureState L2_ALGAE =
      new SuperstructureState(
          Value.of(0.32), // L2_ALGAE_Proportion
          Rotation2d.fromDegrees(
              135), // Wrist_L2_ALGAE_Proportion: 30 - 230 * 0.82 = -158.6 degrees
          RotationsPerSecond.of(10) // Intake_L2_ALGAE_Speed: 1.0 * 20 = 20.0 RPS
          );

  // L3_ALGAE State
  public static final SuperstructureState L3_ALGAE =
      new SuperstructureState(
          Value.of(
              0.8), // L3_ALGAE_Proportion: ProportionToPosition.convertBackwards(32.0) - assuming
          // this equals ~0.8
          Rotation2d.fromDegrees(
              135), // Wrist_L3_ALGAE_Proportion: 30 - 230 * 0.763 = -145.49 degrees
          RotationsPerSecond.of(10) // Intake_L3_ALGAE_Speed: 1.0 * 20 = 20.0 RPS
          );

  public static final SuperstructureState ALGAE_STOW =
      new SuperstructureState(
          Value.of(0.32), // L2_ALGAE_Proportion
          Rotation2d.fromDegrees(
              135), // Wrist_L2_ALGAE_Proportion: 30 - 230 * 0.82 = -158.6 degrees
          RotationsPerSecond.of(0) // Intake_L2_ALGAE_Speed: 1.0 * 20 = 20.0 RPS
          );

  // Barge State
  public static final SuperstructureState BARGE =
      new SuperstructureState(
          Value.of(0.95), // Barge_Proportion
          Rotation2d.fromDegrees(150), // Wrist_Barge_Proportion: 30 - 230 * 0.563 = -99.49 degrees
          RotationsPerSecond.of(3) // Intake_Barge_Speed: 0.1 * 20 = 2.0 RPS
          );

  // Ground_Intake State
  public static final SuperstructureState GROUND_INTAKE =
      new SuperstructureState(
          Value.of(0.0), // Ground_Intake_Proportion
          Rotation2d.fromDegrees(135), // Wrist_Ground_Intake_Proportion: 30 - 230 * 0.845 = -164.35
          // degrees
          RotationsPerSecond.of(-9) // No intake speed specified, assuming 0.0 RPS
          );

  // Homing State
  public static final SuperstructureState HOMING =
      new SuperstructureState(
          Value.of(0.0), // Homing_Proportion
          Rotation2d.fromDegrees(
              135), // Using IDLE wrist proportion: 30 - 230 * 0.414 = -65.22 degrees
          RotationsPerSecond.of(0) // No intake speed specified, assuming 0.0 RPS
          );

  // HP_Lower State (new)
  public static final SuperstructureState HP_LOWER =
      new SuperstructureState(
          Value.of(0.048), // HP_Proportion (same as HP)
          Rotation2d.fromDegrees(40), // Wrist_HP_Lower_Proportion: 30 - 230 * 0.14 = -2.2 degrees
          RotationsPerSecond.of(13) // Intake_HP_Speed: 0.5 * 20 = 10.0 RPS
          );

  public static final SuperstructureState HOMING_READY =
      new SuperstructureState(
          Value.of(0.1), Rotation2d.fromDegrees(135), RotationsPerSecond.zero());

  public static final SuperstructureState REJECT_CORAL =
      new SuperstructureState(Value.of(0), Rotation2d.fromDegrees(150), RotationsPerSecond.of(-5));

  public static final SuperstructureState REJECT_ALGAE =
      new SuperstructureState(Value.of(0), Rotation2d.fromDegrees(135), RotationsPerSecond.of(5));
}
