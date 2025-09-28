package org.team4639.robot.commands;

import static edu.wpi.first.units.Units.Inches;
import static edu.wpi.first.units.Units.Seconds;
import static edu.wpi.first.units.Units.Value;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Transform2d;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import java.util.List;
import org.team4639.robot.commands.superstructure.SetpointSuperstructureCommand;
import org.team4639.robot.constants.FieldConstants;
import org.team4639.robot.modaltriggers.DriveTriggers;
import org.team4639.robot.robot.Subsystems;
import org.team4639.robot.subsystems.superstructure.SuperstructureSetpoints;

public final class AutoCommands {
  public static Command scoreL4(Pose2d pose) {
    return (((Subsystems.drive
                .defer(() -> DriveCommands.PIDToReefPose(pose))
                .deadlineFor((SuperstructureCommands.autoL4Prep())))
            .andThen(
                (Subsystems.drive
                        .defer(() -> DriveCommands.PIDToReefPose(pose))
                        .alongWith(SuperstructureCommands.l4Auto()))
                    .until(Subsystems.wrist::doesNotHaveCoral)))
        .andThen(
            SuperstructureCommands.idle()
                .until(() -> Subsystems.elevator.getPercentage().lte(Value.of(0.5)))));
  }

  public static Command scoreL3(Pose2d pose) {
    return (((Subsystems.drive
                .defer(() -> DriveCommands.PIDToReefPose(pose))
                .deadlineFor(SuperstructureCommands.elevatorReady()))
            .andThen(
                Subsystems.drive
                    .defer(DriveCommands::stopWithX)
                    .alongWith(SuperstructureCommands.l3()))
            .until(Subsystems.wrist::doesNotHaveCoral))
        .andThen(
            SuperstructureCommands.idle()
                .until(() -> Subsystems.elevator.getPercentage().lte(Value.of(0.5)))));
  }

  public static Command scoreL2(Pose2d pose) {
    return (((Subsystems.drive
                .defer(() -> DriveCommands.PIDToReefPose(pose))
                .deadlineFor(SuperstructureCommands.coralStow()))
            .andThen(
                Subsystems.drive
                    .defer(DriveCommands::stopWithX)
                    .alongWith(SuperstructureCommands.l2()))
            .until(Subsystems.wrist::doesNotHaveCoral))
        .andThen(
            SuperstructureCommands.idle()
                .until(() -> Subsystems.elevator.getPercentage().lte(Value.of(0.5)))));
  }

  public static Command scoreBarge(Pose2d pose) {
    return (Subsystems.drive
            .defer(() -> DriveCommands.PIDToPose(pose))
            .deadlineFor(SuperstructureCommands.algaeStow()))
        .andThen(
            (SuperstructureCommands.barge().withExecutionTimeout(Seconds.of(0.75)).flashOnDone())
                .deadlineFor(Subsystems.drive.defer(DriveCommands::stopWithX)));
  }

  public static Command elevatorReady() {
    return new SetpointSuperstructureCommand(
            SuperstructureSetpoints.AUTO_L4_PREP, "AUTO_ELEVATOR_L4_READY")
        .flashOnDone();
  }

  /** Intended to be used after reef center align */
  // TODO: move this somewhere more appropriate since its used in teleop too
  public static Command algaeIntakeSequence() {
    return Subsystems.drive
        .defer(
            () ->
                DriveCommands.PIDToPose(
                    Subsystems.drive
                        .getPose()
                        .nearest(
                            List.of(
                                FieldConstants.TargetPositions.REEF_AB.getPose(),
                                FieldConstants.TargetPositions.REEF_CD.getPose(),
                                FieldConstants.TargetPositions.REEF_EF.getPose(),
                                FieldConstants.TargetPositions.REEF_GH.getPose(),
                                FieldConstants.TargetPositions.REEF_IJ.getPose(),
                                FieldConstants.TargetPositions.REEF_KL.getPose()))
                        .transformBy(
                            new Transform2d(Inches.of(13), Inches.zero(), Rotation2d.kZero))))
        .deadlineFor(SuperstructureCommands.algaeIntake())
        .until(DriveTriggers.joysticksActive)
        .andThen(Commands.waitSeconds(0.2));
  }
}
