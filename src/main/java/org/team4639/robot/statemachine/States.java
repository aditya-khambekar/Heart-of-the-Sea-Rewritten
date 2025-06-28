package org.team4639.robot.statemachine;

import static edu.wpi.first.units.Units.Seconds;

import org.team4639.lib.statebased.State;
import org.team4639.robot.commands.DriveCommands;
import org.team4639.robot.commands.SuperstructureCommands;
import org.team4639.robot.commands.superstructure.MicroAdjustmentCommand;
import org.team4639.robot.constants.Controls;
import org.team4639.robot.constants.FieldConstants;
import org.team4639.robot.modaltriggers.DriveTriggers;
import org.team4639.robot.robot.RobotContainer;
import org.team4639.robot.robot.Subsystems;
import org.team4639.robot.subsystems.superstructure.Superstructure;
import org.team4639.robot.subsystems.superstructure.SuperstructureSetpoints;

public class States {

  public static State IDLE;
  public static State NONE = new State("NONE");
  public static State HP_LEFT;
  public static State HP_RIGHT;
  public static State HP_NODIR;
  public static State INTAKE_LOWER;
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
  public static State MICROADJUSTMENTS;

  public static void initStates() {
    IDLE =
        new State("IDLE")
            .whileTrue(SuperstructureCommands.IDLE)
            .onTrigger(DriveTriggers.closeToRightStation, () -> HP_RIGHT)
            .onTrigger(DriveTriggers.closeToLeftStation, () -> HP_LEFT)
            .onTrigger(Controls.intake, () -> HP_NODIR)
            .withEndCondition(Subsystems.wrist::hasCoral, () -> CORAL_STOW);

    HP_LEFT =
        new State("HP_LEFT")
            .whileTrue(
                DriveCommands.coralStationAlignLeft(Subsystems.drive), SuperstructureCommands.HP)
            .withEndCondition(DriveTriggers.closeToLeftStation.negate(), () -> INTAKE_LOWER)
            .onTrigger(Controls.secondIntake, () -> INTAKE_LOWER)
            .onEmergency(() -> IDLE);

    HP_RIGHT =
        new State("HP_RIGHT")
            .whileTrue(
                DriveCommands.coralStationAlignRight(Subsystems.drive), SuperstructureCommands.HP)
            .withEndCondition(DriveTriggers.closeToRightStation.negate(), () -> INTAKE_LOWER)
            .onTrigger(Controls.secondIntake, () -> INTAKE_LOWER)
            .onEmergency(() -> IDLE);

    HP_NODIR =
        new State("INTAKE_NODIR")
            .whileTrue(SuperstructureCommands.HP)
            .onTrigger(Controls.secondIntake, () -> INTAKE_LOWER);

    INTAKE_LOWER =
        new State("INTAKE_LOWER")
            .whileTrue(SuperstructureCommands.HP_LOWER)
            .withEndCondition(Subsystems.wrist::hasCoral, () -> CORAL_STOW)
            .onEmergency(() -> IDLE);

    CORAL_STOW =
        new State("CORAL_STOW")
            .whileTrue(
                SuperstructureCommands.CORAL_STOW,
                DriveCommands.joystickDriveAtAngle(
                    Subsystems.drive,
                    () -> -RobotContainer.driver.getLeftY(),
                    () -> -RobotContainer.driver.getLeftX(),
                    () ->
                        FieldConstants.getRotationToClosestBranchPosition(
                            Subsystems.drive.getPose())))
            .withEndCondition(Controls.alignLeft, () -> CORAL_SCORE_ALIGN_LEFT)
            .withEndCondition(Controls.alignRight, () -> CORAL_SCORE_ALIGN_RIGHT)
            .withEndCondition(Subsystems.wrist::doesNotHaveCoral, () -> IDLE)
            .onEmergency(() -> REJECT_CORAL);

    CORAL_SCORE_ALIGN_LEFT =
        new State("CORAL_SCORE_ALIGN_LEFT")
            .whileTrue(
                SuperstructureCommands.ELEVATOR_READY,
                Subsystems.drive.defer(() -> DriveCommands.reefAlignLeft(Subsystems.drive)),
                    Subsystems.dashboardOutputs.displayUpcomingReefLevel())
            .withEndCondition(Subsystems.drive::atSetpointTranslation, () -> CHOOSE_CORAL_LEVEL)
            .onTrigger(Controls.alignRight, () -> States.CORAL_SCORE_ALIGN_RIGHT)
            .withEndCondition(Subsystems.wrist::doesNotHaveCoral, () -> IDLE)
            .onEmergency(() -> CORAL_STOW)
            .onAccelerationLimit(() -> CORAL_STOW);

    CORAL_SCORE_ALIGN_RIGHT =
        new State("CORAL_SCORE_ALIGN_RIGHT")
            .whileTrue(
                SuperstructureCommands.ELEVATOR_READY,
                Subsystems.drive.defer(() -> DriveCommands.reefAlignRight(Subsystems.drive)),
                    Subsystems.dashboardOutputs.displayUpcomingReefLevel())
            .withEndCondition(Subsystems.drive::atSetpointTranslation, () -> CHOOSE_CORAL_LEVEL)
            .onTrigger(Controls.alignLeft, () -> States.CORAL_SCORE_ALIGN_LEFT)
            .withEndCondition(Subsystems.wrist::doesNotHaveCoral, () -> IDLE)
            .onEmergency(() -> CORAL_STOW)
            .onAccelerationLimit(() -> CORAL_STOW);

    ALGAE_STOW =
        new State("ALGAE_STOW")
            .whileTrue(SuperstructureCommands.ALGAE_STOW)
            .onEmergency(() -> REJECT_ALGAE);

    ALGAE_SCORE =
        new State("ALGAE_SCORE")
            .whileTrue(SuperstructureCommands.BARGE)
            .onEmergency(() -> ALGAE_STOW)
            .onAccelerationLimit(() -> ALGAE_STOW);

    CHOOSE_CORAL_LEVEL =
        new State("CHOOSE_CORAL_LEVEL")
            .whileTrue(
                SuperstructureCommands.HOLD, Subsystems.drive.run(() -> Subsystems.drive.stop()))
            .withEndCondition(Controls.alignLeft, () -> CORAL_SCORE_ALIGN_LEFT)
            .withEndCondition(Controls.alignRight, () -> CORAL_SCORE_ALIGN_RIGHT)
            .withEndCondition(Subsystems.wrist::doesNotHaveCoral, () -> IDLE)
            .withEndCondition(() -> Subsystems.dashboardOutputs.upcomingReefLevel() == 1, () -> L1_CORAL_SCORE)
            .withEndCondition(() -> Subsystems.dashboardOutputs.upcomingReefLevel() == 2, () -> L2_CORAL_SCORE)
            .withEndCondition(() -> Subsystems.dashboardOutputs.upcomingReefLevel() == 3, () -> L3_CORAL_SCORE)
            .withEndCondition(() -> Subsystems.dashboardOutputs.upcomingReefLevel() == 4, () -> L4_CORAL_SCORE);

    L1_CORAL_SCORE =
        new State("L1_CORAL_SCORE")
            .whileTrue(SuperstructureCommands.L1,
                    Subsystems.reefTracker.scoreL1())
            .withEndCondition(Subsystems.wrist::doesNotHaveCoral, () -> HOMING_READY)
            .onEmergency(() -> CORAL_STOW)
            .onAccelerationLimit(() -> CORAL_STOW);

    L2_CORAL_SCORE =
        new State("L2_CORAL_SCORE")
            .whileTrue(SuperstructureCommands.L2,
                    Subsystems.reefTracker.scoreL2())
            .withEndCondition(Subsystems.wrist::doesNotHaveCoral, () -> HOMING_READY)
            .onEmergency(() -> CORAL_STOW)
            .onAccelerationLimit(() -> CORAL_STOW);

    L3_CORAL_SCORE =
        new State("L3_CORAL_SCORE")
            .whileTrue(SuperstructureCommands.L3,
                    Subsystems.reefTracker.scoreL3())
            .withEndCondition(Subsystems.wrist::doesNotHaveCoral, () -> HOMING_READY)
            .onEmergency(() -> CORAL_STOW)
            .onAccelerationLimit(() -> CORAL_STOW);

    L4_CORAL_SCORE =
        new State("L4_CORAL_SCORE")
            .whileTrue(SuperstructureCommands.L4,
                    Subsystems.reefTracker.scoreL4())
            .withEndCondition(Subsystems.wrist::doesNotHaveCoral, () -> HOMING_READY)
            .onEmergency(() -> CORAL_STOW)
            .onAccelerationLimit(() -> CORAL_STOW);

    HOMING_READY =
        new State("HOMING_READY")
            .whileTrue(SuperstructureCommands.HOMING_READY)
            .withEndCondition(
                () ->
                    Superstructure.atPosition(
                        Superstructure.getSuperstructureState(),
                        SuperstructureSetpoints.HOMING_READY),
                () -> HOMING)
            .onEmergency(() -> IDLE);

    HOMING = new State("HOMING").whileTrue(SuperstructureCommands.HOMING);

    REJECT_CORAL =
        new State("REJECT_CORAL")
            .whileTrue(SuperstructureCommands.REJECT_CORAL)
            .withEndCondition(Subsystems.wrist::doesNotHaveCoral, () -> IDLE);

    REJECT_ALGAE =
        new State("REJECT_ALGAE")
            .whileTrue(SuperstructureCommands.REJECT_ALGAE)
            .withTimeout(Seconds.of(0.5), () -> IDLE);

    MICROADJUSTMENTS = new State("MICROADJUSTMENTS")
            .whileTrue(new MicroAdjustmentCommand(), DriveCommands.stopWithX())
            .onEmergency(() -> CORAL_STOW);
  }
}
