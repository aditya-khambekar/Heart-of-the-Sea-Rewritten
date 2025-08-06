package org.team4639.robot.commands.pathfinding;

import static edu.wpi.first.units.Units.Meters;
import static edu.wpi.first.units.Units.MetersPerSecond;
import static edu.wpi.first.units.Units.MetersPerSecondPerSecond;
import static edu.wpi.first.units.Units.RotationsPerSecond;
import static edu.wpi.first.units.Units.RotationsPerSecondPerSecond;
import static edu.wpi.first.units.Units.Volts;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.path.PathConstraints;
import com.pathplanner.lib.path.PathPlannerPath;
import com.pathplanner.lib.util.FileVersionException;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import java.io.IOException;
import java.util.ArrayList;
import java.util.List;
import org.json.simple.parser.ParseException;
import org.team4639.lib.util.PoseUtil;
import org.team4639.robot.commands.DriveCommands;
import org.team4639.robot.robot.Subsystems;

public class TeleopPathGenerator {
  public static Command pathfindToReef(Pose2d startingPose, Pose2d endingPose) {
    var starting = ReefPathLocations.getClosest(startingPose);
    var ending = ReefPathLocations.getClosest(endingPose);

    if (starting == ending
        || starting.leftReef.get() == ending
        || starting.rightReef.get() == ending) {
      return (Subsystems.drive
              .defer(
                  () ->
                      AutoBuilder.pathfindToPose(
                          endingPose,
                          new PathConstraints(
                              MetersPerSecond.of(3),
                              MetersPerSecondPerSecond.of(6),
                              RotationsPerSecond.of(2),
                              RotationsPerSecondPerSecond.of(4),
                              Volts.of(12),
                              true),
                          MetersPerSecond.of(1)))
              .until(
                  () ->
                      PoseUtil.getDistance(Subsystems.drive.getPose(), endingPose).in(Meters) < 2.5)
              .andThen(Subsystems.drive.defer(() -> DriveCommands.PIDToReefPose(endingPose))))
          .beforeStarting(Subsystems.reefTracker.setCurrentReefPoseCommand(endingPose));
    }

    var alignDistanceMeters =
        PoseUtil.getDistance(ending.getPoseOfStartPath(), endingPose).in(Meters) * 1.5;

    return getPathCommand(starting, ending)
        .until(
            () ->
                PoseUtil.getDistance(Subsystems.drive.getPose(), endingPose).in(Meters)
                    < alignDistanceMeters)
        .andThen(Subsystems.drive.defer(() -> DriveCommands.PIDToReefPose(endingPose)))
        .beforeStarting(Subsystems.reefTracker.setCurrentReefPoseCommand(endingPose));
  }

  public static Command pathfindTo(Pose2d startingPose, Pose2d endingPose) {
    var starting = ReefPathLocations.getClosest(startingPose);
    var ending = ReefPathLocations.getClosest(endingPose);

    if (starting == ending
        || starting.leftReef.get() == ending
        || starting.rightReef.get() == ending) {
      return (Subsystems.drive
          .defer(
              () ->
                  AutoBuilder.pathfindToPose(
                      endingPose,
                      new PathConstraints(
                          MetersPerSecond.of(3),
                          MetersPerSecondPerSecond.of(6),
                          RotationsPerSecond.of(2),
                          RotationsPerSecondPerSecond.of(4),
                          Volts.of(12),
                          true),
                      MetersPerSecond.of(1)))
          .until(
              () -> PoseUtil.getDistance(Subsystems.drive.getPose(), endingPose).in(Meters) < 1.5)
          .andThen(Subsystems.drive.defer(() -> DriveCommands.PIDToPose(endingPose))));
    }

    var alignDistanceMeters =
        PoseUtil.getDistance(ending.getPoseOfStartPath(), endingPose).in(Meters) * 1.5;

    return getPathCommand(starting, ending)
        .until(
            () ->
                PoseUtil.getDistance(Subsystems.drive.getPose(), endingPose).in(Meters)
                    < alignDistanceMeters)
        .andThen(Subsystems.drive.defer(() -> DriveCommands.PIDToPose(endingPose)));
  }

  public static Command getPathCommand(ReefPathLocations starting, ReefPathLocations ending) {
    List<ReefPathLocations> locations = chainPaths(starting, ending);
    List<Command> commands = new ArrayList<>();
    for (int i = 0; i < locations.size() - 1; i++) {
      try {
        commands.add(
            AutoBuilder.followPath(
                PathPlannerPath.fromPathFile(
                    locations.get(i).name + "-" + locations.get(i + 1).name)));
      } catch (FileVersionException | IOException | ParseException e) {
        // TODO Auto-generated catch block
        e.printStackTrace();
      }
    }
    return Commands.sequence(commands.toArray(new Command[0]));
  }

  public static List<ReefPathLocations> chainPaths(
      ReefPathLocations starting, ReefPathLocations ending) {
    List<ReefPathLocations> rightDaemon = new ArrayList<>();
    List<ReefPathLocations> leftDaemon = new ArrayList<>();

    rightDaemon.add(starting);
    leftDaemon.add(starting);

    for (int i = 0; i < 8; i++) {
      var rightAddition = rightDaemon.get(rightDaemon.size() - 1).rightReef.get();
      rightDaemon.add(rightAddition);

      var leftAddition = leftDaemon.get(leftDaemon.size() - 1).leftReef.get();
      leftDaemon.add(leftAddition);

      if (rightAddition == ending) return rightDaemon;
      if (leftAddition == ending) {
        return leftDaemon;
      }
    }
    System.out.println("Shit fucked up");
    return List.of();
  }
}
