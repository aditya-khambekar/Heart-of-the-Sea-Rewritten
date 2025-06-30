package org.team4639.robot.commands;

import static edu.wpi.first.units.Units.Inches;
import static edu.wpi.first.units.Units.Seconds;
import static edu.wpi.first.units.Units.Value;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.path.PathPlannerPath;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Transform2d;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import java.util.List;
import java.util.Map;
import org.team4639.robot.commands.superstructure.SuperstructureCommand;
import org.team4639.robot.constants.FieldConstants;
import org.team4639.robot.robot.Subsystems;
import org.team4639.robot.subsystems.superstructure.Superstructure;
import org.team4639.robot.subsystems.superstructure.SuperstructureSetpoints;

public class AutoCommands {
  public static Command scoreL4(Pose2d pose) {
    return (Subsystems.reefTracker.setCurrentReefPoseCommand(pose))
        .andThen(
            ((Subsystems.drive
                        .defer(
                            () ->
                                DriveCommands.PIDtoReefWithVelocityReset(
                                    Subsystems.drive, Subsystems.drive.getPose(), pose))
                        .deadlineFor(SuperstructureCommands.elevatorReady()))
                    .andThen(
                        Subsystems.drive
                            .defer(DriveCommands::stopWithX)
                            .alongWith(SuperstructureCommands.l4()))
                    .until(Subsystems.wrist::doesNotHaveCoral))
                .andThen(
                    SuperstructureCommands.idle()
                        .until(() -> Subsystems.elevator.getPercentage().lte(Value.of(0.5)))))
        .finallyDo(Subsystems.reefTracker::scoreL4Raw);
  }

  public static Command scoreBarge(Pose2d pose) {
    return (Subsystems.drive
            .defer(
                () ->
                    DriveCommands.PIDtowithVelocityReset(
                        Subsystems.drive, Subsystems.drive.getPose(), pose))
            .deadlineFor(SuperstructureCommands.algaeStow()))
        .andThen(
            (SuperstructureCommands.barge().withExecutionTimeout(Seconds.of(0.75)).flashOnDone())
                .deadlineFor(Subsystems.drive.defer(DriveCommands::stopWithX)));
  }

  public static Command intakeLeft() {
    return (DriveCommands.coralStationAlignLeft(Subsystems.drive)
            .deadlineFor(SuperstructureCommands.hpLower()))
        .andThen(DriveCommands.stopWithX().alongWith(SuperstructureCommands.hpLower()))
        .until(Subsystems.wrist::hasCoral);
  }

  public static Command intakeRight() {
    return (DriveCommands.coralStationAlignRight(Subsystems.drive)
            .deadlineFor(SuperstructureCommands.hpLower()))
        .andThen(DriveCommands.stopWithX().alongWith(SuperstructureCommands.hpLower()))
        .until(Subsystems.wrist::hasCoral);
  }

  public static Command elevatorReady() {
    return new SuperstructureCommand(SuperstructureSetpoints.AUTO_ELEVATOR_L4_READY).flashOnDone();
  }

  private static Map<Boolean, Command> pathingSuperstructureCommandMap() {
    return Map.ofEntries(
        Map.entry(true, SuperstructureCommands.elevatorReady()),
        Map.entry(false, SuperstructureCommands.hpLower()));
  }

  private static Map<Boolean, Command> pathingSuperstructureCommandMapAlgae() {
    return Map.ofEntries(
        Map.entry(true, SuperstructureCommands.algaeStow()),
        Map.entry(false, SuperstructureCommands.elevatorReady()));
  }

  public static Command path(PathPlannerPath path) {
    return AutoBuilder.followPath(path)
        .deadlineFor(
            Commands.select(pathingSuperstructureCommandMap(), Subsystems.wrist::hasCoral));
  }

  public static Command pathForAlgae(PathPlannerPath path, boolean hasAlgae) {
    return AutoBuilder.followPath(path)
        .deadlineFor(Commands.select(pathingSuperstructureCommandMapAlgae(), () -> hasAlgae));
  }

  /**
   * Intended to be used after reef center align
   *
   * @return
   */
  public static Command algaeIntakeSequence() {
    return Subsystems.drive
        .defer(
            () ->
                DriveCommands.PIDtowithVelocityReset(
                    Subsystems.drive,
                    Subsystems.drive.getPose(),
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
                            new Transform2d(Inches.of(10), Inches.zero(), Rotation2d.kZero))))
        .deadlineFor(SuperstructureCommands.algaeIntake())
        .andThen(
            Subsystems.drive
                .defer(
                    () ->
                        DriveCommands.PIDtowithVelocityReset(
                            Subsystems.drive,
                            Subsystems.drive.getPose(),
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
                                    new Transform2d(
                                        Inches.of(-5), Inches.zero(), Rotation2d.kZero))))
                .andThen(
                    (DriveCommands.stopWithX().alongWith(SuperstructureCommands.algaeStow()))
                        .until(
                            () -> Superstructure.atPosition(SuperstructureSetpoints.ALGAE_STOW))));
  }
}
