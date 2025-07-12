package org.team4639.robot.commands;

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
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Transform2d;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import java.io.IOException;
import java.util.ArrayList;
import java.util.List;
import java.util.function.Supplier;
import org.json.simple.parser.ParseException;
import org.team4639.lib.util.PoseUtil;
import org.team4639.robot.constants.reefscape.TargetPositions;
import org.team4639.robot.robot.Subsystems;

// TODO: there is probably a better way to do this
public final class TeleopPathCommands {
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
              () -> PoseUtil.getDistance(Subsystems.drive.getPose(), endingPose).in(Meters) < 2.5)
          .andThen(Subsystems.drive.defer(() -> DriveCommands.PIDToReef(endingPose))));
    }

    var alignDistanceMeters =
        PoseUtil.getDistance(ending.getPoseOfStartPath(), endingPose).in(Meters) * 1.5;

    return getPathCommand(starting, ending)
        .until(
            () ->
                PoseUtil.getDistance(Subsystems.drive.getPose(), endingPose).in(Meters)
                    < alignDistanceMeters)
        .andThen(Subsystems.drive.defer(() -> DriveCommands.PIDToReef(endingPose)));
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
          .andThen(Subsystems.drive.defer(() -> DriveCommands.Evergreen.PIDTo(endingPose))));
    }

    var alignDistanceMeters =
        PoseUtil.getDistance(ending.getPoseOfStartPath(), endingPose).in(Meters) * 1.5;

    return getPathCommand(starting, ending)
        .until(
            () ->
                PoseUtil.getDistance(Subsystems.drive.getPose(), endingPose).in(Meters)
                    < alignDistanceMeters)
        .andThen(Subsystems.drive.defer(() -> DriveCommands.Evergreen.PIDTo(endingPose)));
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

  public static class ReefPathLocations {
    public static final ReefPathLocations AB =
        new ReefPathLocations(
            TargetPositions.REEF_AB,
            TargetPositions.REEF_A,
            TargetPositions.REEF_B,
            () -> TeleopPathCommands.ReefPathLocations.KL,
            () -> TeleopPathCommands.ReefPathLocations.CD,
            "AB");
    public static final ReefPathLocations CD =
        new ReefPathLocations(
            TargetPositions.REEF_CD,
            TargetPositions.REEF_C,
            TargetPositions.REEF_D,
            () -> TeleopPathCommands.ReefPathLocations.AB,
            () -> TeleopPathCommands.ReefPathLocations.EF,
            "CD");
    public static final ReefPathLocations EF =
        new ReefPathLocations(
            TargetPositions.REEF_EF,
            TargetPositions.REEF_E,
            TargetPositions.REEF_F,
            () -> TeleopPathCommands.ReefPathLocations.CD,
            () -> TeleopPathCommands.ReefPathLocations.GH,
            "EF");
    public static final ReefPathLocations GH =
        new ReefPathLocations(
            TargetPositions.REEF_GH,
            TargetPositions.REEF_G,
            TargetPositions.REEF_H,
            () -> TeleopPathCommands.ReefPathLocations.EF,
            () -> TeleopPathCommands.ReefPathLocations.IJ,
            "GH");
    public static final ReefPathLocations IJ =
        new ReefPathLocations(
            TargetPositions.REEF_IJ,
            TargetPositions.REEF_I,
            TargetPositions.REEF_J,
            () -> TeleopPathCommands.ReefPathLocations.GH,
            () -> TeleopPathCommands.ReefPathLocations.KL,
            "IJ");
    public static final ReefPathLocations KL =
        new ReefPathLocations(
            TargetPositions.REEF_KL,
            TargetPositions.REEF_K,
            TargetPositions.REEF_L,
            () -> TeleopPathCommands.ReefPathLocations.IJ,
            () -> TeleopPathCommands.ReefPathLocations.AB,
            "KL");

    public static List<ReefPathLocations> entries = List.of(AB, CD, EF, GH, IJ, KL);

    protected TargetPositions center;
    protected TargetPositions left;
    protected TargetPositions right;
    protected Supplier<ReefPathLocations> rightReef;
    protected Supplier<ReefPathLocations> leftReef;
    protected String name;

    public ReefPathLocations(
        TargetPositions center,
        TargetPositions left,
        TargetPositions right,
        Supplier<ReefPathLocations> leftReef,
        Supplier<ReefPathLocations> rightReef,
        String name) {
      this.center = center;
      this.left = left;
      this.right = right;
      this.leftReef = leftReef;
      this.rightReef = rightReef;
      this.name = name;
    }

    public Pose2d getPoseOfStartPath() {
      return center.getPose().transformBy(new Transform2d(-0.9, 0, Rotation2d.kZero));
    }

    public static ReefPathLocations getClosest(Pose2d pose) {
      var nearestPose = pose.nearest(entries.stream().map(x -> x.getPoseOfStartPath()).toList());
      return entries.stream()
          .filter(x -> x.getPoseOfStartPath().equals(nearestPose))
          .findAny()
          .get();
    }
  }
}
