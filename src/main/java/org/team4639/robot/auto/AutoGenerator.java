package org.team4639.robot.auto;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.path.PathPlannerPath;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import java.util.ArrayList;
import java.util.List;
import org.team4639.robot.commands.AutoCommands;
import org.team4639.robot.commands.DriveCommands;
import org.team4639.robot.commands.SuperstructureCommands;
import org.team4639.robot.constants.FieldConstants;
import org.team4639.robot.robot.Subsystems;

public class AutoGenerator {

  public static enum Location {
    RS,
    LS,
    MS,
    RHP,
    LHP,
    A,
    B,
    C,
    D,
    E,
    F,
    G,
    H,
    I,
    J,
    K,
    L,
    ALGH,
    ALIJ,
    ALGSC1,
    ALGSC2,
    ALGSC3;
  }

  public static enum PathType {
    OUT_CORAL,
    OUT_ALGAE,
    IN_CORAL,
    IN_ALGAE
  }

  public static PathType getPathType(Location endingLocation) {
    return switch (endingLocation) {
      case A, B, C, D, E, F, G, H, I, J, K, L -> PathType.OUT_CORAL;
      case ALGSC1, ALGSC2, ALGSC3 -> PathType.OUT_ALGAE;
      case RHP, LHP -> PathType.IN_CORAL;
      case ALGH, ALIJ -> PathType.IN_ALGAE;
      default -> throw new IllegalArgumentException("Unexpected value: " + endingLocation);
    };
  }

  public static Command compileAuto(Location... locations) {
    List<Command> commands = new ArrayList<>();
    for (int i = 0; i < locations.length - 1; i++) {
      commands.add(getSegmentCommand(locations[i], locations[i + 1], i == 0));
    }
    return Commands.sequence(commands.toArray(new Command[0]));
  }

  public static Command getSegmentCommand(
      Location starting, Location ending, boolean isFirstCommand) {
    try {
      var path = PathPlannerPath.fromChoreoTrajectory(starting + "-" + ending);
      var pathCommand = AutoBuilder.followPath(path);

      pathCommand =
          isFirstCommand
              ? Commands.runOnce(
                      () ->
                          Subsystems.drive.setPose(
                              path.getStartingHolonomicPose().orElse(new Pose2d())))
                  .andThen(pathCommand)
              : pathCommand;

      return switch (getPathType(ending)) {
        case IN_CORAL -> (pathCommand
                .andThen(
                    switch (ending) {
                      case LHP -> DriveCommands.HPLeftAlign(Subsystems.drive);
                      case RHP -> DriveCommands.HPRightAlign(Subsystems.drive);
                      default -> throw new IllegalArgumentException("what the fuck");
                    })
                .andThen(DriveCommands.stopWithX().withTimeout(0.5)))
            .deadlineFor(SuperstructureCommands.hp());
        case OUT_CORAL -> (pathCommand.deadlineFor(SuperstructureCommands.elevatorReady()))
            .andThen(
                switch (ending) {
                  case A -> AutoCommands.scoreL4(FieldConstants.TargetPositions.REEF_A.getPose());
                  case B -> AutoCommands.scoreL4(FieldConstants.TargetPositions.REEF_B.getPose());
                  case C -> AutoCommands.scoreL4(FieldConstants.TargetPositions.REEF_C.getPose());
                  case D -> AutoCommands.scoreL4(FieldConstants.TargetPositions.REEF_D.getPose());
                  case E -> AutoCommands.scoreL4(FieldConstants.TargetPositions.REEF_E.getPose());
                  case F -> AutoCommands.scoreL4(FieldConstants.TargetPositions.REEF_F.getPose());
                  case G -> AutoCommands.scoreL4(FieldConstants.TargetPositions.REEF_G.getPose());
                  case H -> AutoCommands.scoreL4(FieldConstants.TargetPositions.REEF_H.getPose());
                  case I -> AutoCommands.scoreL4(FieldConstants.TargetPositions.REEF_I.getPose());
                  case J -> AutoCommands.scoreL4(FieldConstants.TargetPositions.REEF_J.getPose());
                  case K -> AutoCommands.scoreL4(FieldConstants.TargetPositions.REEF_K.getPose());
                  case L -> AutoCommands.scoreL4(FieldConstants.TargetPositions.REEF_L.getPose());
                  default -> throw new IllegalArgumentException("what the fuck");
                });
        case IN_ALGAE -> (pathCommand.deadlineFor(SuperstructureCommands.elevatorReady()))
            .andThen(AutoCommands.algaeIntakeSequence());
        case OUT_ALGAE -> (pathCommand.deadlineFor(SuperstructureCommands.algaeStow()))
            .andThen(
                switch (ending) {
                  case ALGSC1 -> AutoCommands.scoreBarge(
                      FieldConstants.TargetPositions.BARGE_FARCAGE.getPose());
                  case ALGSC2 -> AutoCommands.scoreBarge(
                      FieldConstants.TargetPositions.BARGE_MIDDLECAGE.getPose());
                  case ALGSC3 -> AutoCommands.scoreBarge(
                      FieldConstants.TargetPositions.BARGE_CLOSECAGE.getPose());
                  default -> throw new IllegalArgumentException("what the fuck");
                });
      };
    } catch (Exception e) {
      e.printStackTrace();
      return Commands.none();
    }
  }
}
