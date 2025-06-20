package org.team4639.robot.statemachine;

import static edu.wpi.first.units.Units.Seconds;

import org.team4639.robot.commands.DriveCommands;
import org.team4639.robot.commands.SuperstructureCommands;
import org.team4639.robot.constants.Controls;
import org.team4639.robot.modaltriggers.DriveTriggers;
import org.team4639.robot.robot.Subsystems;
import org.team4639.robot.subsystems.superstructure.Superstructure;
import org.team4639.robot.subsystems.superstructure.SuperstructureSetpoints;

public class States {

  public static State IDLE;
  public static State NONE = new State("NONE");
  public static State INTAKE_LEFT;
  public static State INTAKE_RIGHT;
  public static State INTAKE_NODIR;
  public static State INTAKE_II;
  public static State CORAL_STOW;
  public static State CORAL_SCORE_ALIGN_LEFT;
  public static State CORAL_SCORE_ALIGN_RIGHT;
  public static State ALGAE_STOW;
  public static State ALGAE_SCORE;
  public static State CHOOSE_CORAL_LEVEL;
  public static State L1_CORAL_SCORE;
  public static State L2_CORAL_SCORE;
  public static State L3_CORAL_SCORE;
  public static State L4_CORAL_SCORE;
  public static State HOMING_READY;
  public static State HOMING;
  public static State REJECT_CORAL;
  public static State REJECT_ALGAE;

  public static void initStates() {
    IDLE =
        new State("IDLE")
            .whileTrue(SuperstructureCommands.IDLE)
            .withEndCondition(DriveTriggers.closeToRightStation, () -> INTAKE_RIGHT)
            .withEndCondition(DriveTriggers.closeToLeftStation, () -> INTAKE_LEFT)
            .withEndCondition(Controls.intake, () -> INTAKE_NODIR);

    INTAKE_LEFT =
        new State("INTAKE_LEFT")
            .whileTrue(
                DriveCommands.coralStationAlignLeft(Subsystems.drive),
                SuperstructureCommands.INTAKE)
            .withEndCondition(DriveTriggers.closeToLeftStation.negate(), () -> INTAKE_II)
            .withEndCondition(Controls.secondIntake, () -> INTAKE_II)
            .onEmergency(() -> IDLE);

    INTAKE_RIGHT =
        new State("INTAKE_RIGHT")
            .whileTrue(
                DriveCommands.coralStationAlignRight(Subsystems.drive),
                SuperstructureCommands.INTAKE)
            .withEndCondition(DriveTriggers.closeToRightStation.negate(), () -> INTAKE_II)
            .withEndCondition(Controls.secondIntake, () -> INTAKE_II)
            .onEmergency(() -> IDLE);

    INTAKE_NODIR =
        new State("INTAKE_NODIR")
            .whileTrue(SuperstructureCommands.INTAKE)
            .withEndCondition(Controls.secondIntake, () -> INTAKE_II);

    INTAKE_II =
        new State("INTAKE_II")
            .whileTrue(SuperstructureCommands.INTAKE_II)
            .withEndCondition(Subsystems.wrist::hasCoral, () -> CORAL_STOW)
            .onEmergency(() -> IDLE);

    CORAL_STOW =
        new State("CORAL_STOW")
            .whileTrue(SuperstructureCommands.CORAL_STOW)
            .withEndCondition(Controls.alignLeft, () -> CORAL_SCORE_ALIGN_LEFT)
            .withEndCondition(Controls.alignRight, () -> CORAL_SCORE_ALIGN_RIGHT)
            .onEmergency(() -> REJECT_CORAL);

    CORAL_SCORE_ALIGN_LEFT =
        new State("CORAL_SCORE_ALIGN_LEFT")
            .whileTrue(
                SuperstructureCommands.ELEVATOR_READY,
                DriveCommands.reefAlignLeft(Subsystems.drive))
            .withEndCondition(
                () -> Subsystems.drive.getCurrentCommand().getName().endsWith("Joystick"),
                () -> CHOOSE_CORAL_LEVEL)
            .onEmergency(() -> CORAL_STOW)
            .onAccelerationLimit(() -> CORAL_STOW);

    CORAL_SCORE_ALIGN_RIGHT =
        new State("CORAL_SCORE_ALIGN_RIGHT")
            .whileTrue(
                SuperstructureCommands.ELEVATOR_READY,
                DriveCommands.reefAlignRight(Subsystems.drive))
            .withEndCondition(
                () -> Subsystems.drive.getCurrentCommand().getName().endsWith("Joystick"),
                () -> CHOOSE_CORAL_LEVEL)
            .onEmergency(() -> CORAL_STOW)
            .onAccelerationLimit(() -> CORAL_STOW);

    ALGAE_STOW =
        new State("ALGAE_STOW")
            .whileTrue(SuperstructureCommands.ALGAE_STOW)
            .onEmergency(() -> REJECT_ALGAE);

    ALGAE_SCORE =
        new State("ALGAE_SCORE")
            .whileTrue(SuperstructureCommands.ALGAE_SCORE)
            .onEmergency(() -> ALGAE_STOW)
            .onAccelerationLimit(() -> ALGAE_STOW);

    CHOOSE_CORAL_LEVEL =
        new State("CHOOSE_CORAL_LEVEL")
            .whileTrue(
                SuperstructureCommands.HOLD, Subsystems.drive.run(() -> Subsystems.drive.stop()))
            .withEndCondition(Controls.L1Coral, () -> L1_CORAL_SCORE)
            .withEndCondition(Controls.L2Coral, () -> L2_CORAL_SCORE)
            .withEndCondition(Controls.L3Coral, () -> L3_CORAL_SCORE)
            .withEndCondition(Controls.L4Coral, () -> L4_CORAL_SCORE);

    L1_CORAL_SCORE =
        new State("L1_CORAL_SCORE")
            .whileTrue(SuperstructureCommands.L1)
            .withEndCondition(Subsystems.wrist::doesNotHaveCoral, () -> HOMING_READY);

    L2_CORAL_SCORE =
        new State("L2_CORAL_SCORE")
            .whileTrue(SuperstructureCommands.L2)
            .withEndCondition(Subsystems.wrist::doesNotHaveCoral, () -> HOMING_READY);

    L3_CORAL_SCORE =
        new State("L3_CORAL_SCORE")
            .whileTrue(SuperstructureCommands.L3)
            .withEndCondition(Subsystems.wrist::doesNotHaveCoral, () -> HOMING_READY);

    L4_CORAL_SCORE =
        new State("L4_CORAL_SCORE")
            .whileTrue(SuperstructureCommands.L4)
            .withEndCondition(Subsystems.wrist::doesNotHaveCoral, () -> HOMING_READY);

    HOMING_READY =
        new State("HOMING_READY")
            .whileTrue(SuperstructureCommands.HOMING_READY)
            .withEndCondition(
                () ->
                    Superstructure.atPosition(
                        Superstructure.getSuperstructureState(),
                        SuperstructureSetpoints.homing_ready),
                () -> HOMING);

    HOMING = new State("HOMING").whileTrue(SuperstructureCommands.HOMING);

    REJECT_CORAL =
        new State("REJECT_CORAL")
            .whileTrue(SuperstructureCommands.REJECT_CORAL)
            .withEndCondition(Subsystems.wrist::doesNotHaveCoral, () -> IDLE);

    REJECT_ALGAE =
        new State("REJECT_ALGAE")
            .whileTrue(SuperstructureCommands.REJECT_ALGAE)
            .withTimeout(Seconds.of(0.5), () -> IDLE);
  }
}
