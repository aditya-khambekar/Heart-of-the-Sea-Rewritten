package org.team4639.robot.commands;

import static edu.wpi.first.units.Units.Value;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.path.PathPlannerPath;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import java.util.Map;
import org.team4639.robot.commands.superstructure.SuperstructureCommand;
import org.team4639.robot.robot.Subsystems;
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
                        .until(() -> Subsystems.elevator.getPercentage().lte(Value.of(0.4)))))
        .finallyDo(Subsystems.reefTracker::scoreL4Raw);
  }

  public static Command intakeLeft() {
    return DriveCommands.coralStationAlignLeft(Subsystems.drive)
        .andThen(DriveCommands.stopWithX().alongWith(SuperstructureCommands.hpLower()))
        .until(Subsystems.wrist::hasCoral);
  }

  public static Command intakeRight() {
    return DriveCommands.coralStationAlignRight(Subsystems.drive)
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

  public static Command path(PathPlannerPath path) {
    return AutoBuilder.followPath(path)
        .deadlineFor(
            Commands.select(pathingSuperstructureCommandMap(), Subsystems.wrist::hasCoral));
  }
}
