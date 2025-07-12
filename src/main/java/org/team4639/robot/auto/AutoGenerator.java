package org.team4639.robot.auto;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.path.PathPlannerPath;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import java.util.ArrayList;
import java.util.List;
import java.util.Objects;
import org.team4639.robot.commands.AutoCommands;
import org.team4639.robot.commands.DriveCommands;
import org.team4639.robot.commands.SuperstructureCommands;
import org.team4639.robot.constants.reefscape.TargetPositions;
import org.team4639.robot.robot.Subsystems;

public final class AutoGenerator {

  public static enum Location {
    RS,
    LS,
    MS,
    RHP,
    LHP,
    A4("A"),
    A3("A"),
    A2("A"),
    B4("B"),
    B3("B"),
    B2("B"),
    C4("C"),
    C3("C"),
    C2("C"),
    D4("D"),
    D3("D"),
    D2("D"),
    E4("E"),
    E3("E"),
    E2("E"),
    F4("F"),
    F3("F"),
    F2("F"),
    G4("G"),
    G3("G"),
    G2("G"),
    H4("H"),
    H3("H"),
    H2("H"),
    I4("I"),
    I3("I"),
    I2("I"),
    J4("J"),
    J3("J"),
    J2("J"),
    K4("K"),
    K3("K"),
    K2("K"),
    L4("L"),
    L3("L"),
    L2("L"),
    ALGH,
    ALIJ,
    ALGSC1,
    ALGSC2,
    ALGSC3;

    private String name = null;

    Location() {}

    Location(String name) {
      this.name = name;
    }

    /**
     * @return The name of this Location as would be found in a pathplanner path. Specifically, this
     *     will omit the level on the coral locations.
     */
    public String getName() {
      return Objects.requireNonNullElse(name, toString());
    }
  }

  public static enum PathType {
    OUT_CORAL,
    OUT_ALGAE,
    IN_CORAL,
    IN_ALGAE
  }

  public static PathType getPathType(Location endingLocation) {
    return switch (endingLocation) {
      case A4,
          B4,
          C4,
          D4,
          E4,
          F4,
          G4,
          H4,
          I4,
          J4,
          K4,
          L4,
          A3,
          B3,
          C3,
          D3,
          E3,
          F3,
          G3,
          H3,
          I3,
          J3,
          K3,
          L3,
          A2,
          B2,
          C2,
          D2,
          E2,
          F2,
          G2,
          H2,
          I2,
          J2,
          K2,
          L2 -> PathType.OUT_CORAL;
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
                      case LHP -> DriveCommands.HPStationAlignLeft();
                      case RHP -> DriveCommands.HPStationAlignRight();
                      default -> throw new IllegalArgumentException("what the fuck");
                    })
                .andThen(DriveCommands.Evergreen.stopWithX().withTimeout(0.5)))
            .deadlineFor(SuperstructureCommands.hp());
        case OUT_CORAL -> (pathCommand.deadlineFor(SuperstructureCommands.elevatorReady()))
            .andThen(
                switch (ending) {
                    // L4
                  case A4 -> AutoCommands.scoreL4(TargetPositions.REEF_A.getPose());
                  case B4 -> AutoCommands.scoreL4(TargetPositions.REEF_B.getPose());
                  case C4 -> AutoCommands.scoreL4(TargetPositions.REEF_C.getPose());
                  case D4 -> AutoCommands.scoreL4(TargetPositions.REEF_D.getPose());
                  case E4 -> AutoCommands.scoreL4(TargetPositions.REEF_E.getPose());
                  case F4 -> AutoCommands.scoreL4(TargetPositions.REEF_F.getPose());
                  case G4 -> AutoCommands.scoreL4(TargetPositions.REEF_G.getPose());
                  case H4 -> AutoCommands.scoreL4(TargetPositions.REEF_H.getPose());
                  case I4 -> AutoCommands.scoreL4(TargetPositions.REEF_I.getPose());
                  case J4 -> AutoCommands.scoreL4(TargetPositions.REEF_J.getPose());
                  case K4 -> AutoCommands.scoreL4(TargetPositions.REEF_K.getPose());
                  case L4 -> AutoCommands.scoreL4(TargetPositions.REEF_L.getPose());
                    // L3
                  case A3 -> AutoCommands.scoreL3(TargetPositions.REEF_A.getPose());
                  case B3 -> AutoCommands.scoreL3(TargetPositions.REEF_B.getPose());
                  case C3 -> AutoCommands.scoreL3(TargetPositions.REEF_C.getPose());
                  case D3 -> AutoCommands.scoreL3(TargetPositions.REEF_D.getPose());
                  case E3 -> AutoCommands.scoreL3(TargetPositions.REEF_E.getPose());
                  case F3 -> AutoCommands.scoreL3(TargetPositions.REEF_F.getPose());
                  case G3 -> AutoCommands.scoreL3(TargetPositions.REEF_G.getPose());
                  case H3 -> AutoCommands.scoreL3(TargetPositions.REEF_H.getPose());
                  case I3 -> AutoCommands.scoreL3(TargetPositions.REEF_I.getPose());
                  case J3 -> AutoCommands.scoreL3(TargetPositions.REEF_J.getPose());
                  case K3 -> AutoCommands.scoreL3(TargetPositions.REEF_K.getPose());
                  case L3 -> AutoCommands.scoreL3(TargetPositions.REEF_L.getPose());
                    // L2
                  case A2 -> AutoCommands.scoreL2(TargetPositions.REEF_A.getPose());
                  case B2 -> AutoCommands.scoreL2(TargetPositions.REEF_B.getPose());
                  case C2 -> AutoCommands.scoreL2(TargetPositions.REEF_C.getPose());
                  case D2 -> AutoCommands.scoreL2(TargetPositions.REEF_D.getPose());
                  case E2 -> AutoCommands.scoreL2(TargetPositions.REEF_E.getPose());
                  case F2 -> AutoCommands.scoreL2(TargetPositions.REEF_F.getPose());
                  case G2 -> AutoCommands.scoreL2(TargetPositions.REEF_G.getPose());
                  case H2 -> AutoCommands.scoreL2(TargetPositions.REEF_H.getPose());
                  case I2 -> AutoCommands.scoreL2(TargetPositions.REEF_I.getPose());
                  case J2 -> AutoCommands.scoreL2(TargetPositions.REEF_J.getPose());
                  case K2 -> AutoCommands.scoreL2(TargetPositions.REEF_K.getPose());
                  case L2 -> AutoCommands.scoreL2(TargetPositions.REEF_L.getPose());
                  default -> throw new IllegalArgumentException("what the fuck");
                });
        case IN_ALGAE -> (pathCommand.deadlineFor(SuperstructureCommands.elevatorReady()))
            .andThen(AutoCommands.algaeIntakeSequence());
        case OUT_ALGAE -> (pathCommand.deadlineFor(SuperstructureCommands.algaeStow()))
            .andThen(
                switch (ending) {
                  case ALGSC1 -> AutoCommands.scoreBarge(TargetPositions.BARGE_FARCAGE.getPose());
                  case ALGSC2 -> AutoCommands.scoreBarge(
                      TargetPositions.BARGE_MIDDLECAGE.getPose());
                  case ALGSC3 -> AutoCommands.scoreBarge(TargetPositions.BARGE_CLOSECAGE.getPose());
                  default -> throw new IllegalArgumentException("what the fuck");
                });
      };
    } catch (Exception e) {
      e.printStackTrace();
      return Commands.none();
    }
  }
}
