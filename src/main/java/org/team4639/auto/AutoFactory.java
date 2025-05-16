package org.team4639.auto;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.path.PathPlannerPath;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import java.io.IOException;
import java.util.ArrayList;
import java.util.List;
import org.json.simple.parser.ParseException;
import org.team4639._robot.Subsystems;
import org.team4639.commands.DriveCommands;
import org.team4639.commands.SuperstructureCommands;
import org.team4639.constants.FieldConstants;
import org.team4639.subsystems.elevator.ElevatorConstants;

public class AutoFactory {

  public static Command alignAndScore(FieldConstants.TargetPositions target, int level) {
    return SuperstructureCommands.score(
        switch (level) {
          case 1 -> ElevatorConstants.Setpoints.L1_PROPORTION;
          case 2 -> ElevatorConstants.Setpoints.L2_PROPORTION;
          case 3 -> ElevatorConstants.Setpoints.L3_PROPORTION;
          case 4 -> ElevatorConstants.Setpoints.L4_PROPORTION;
          default -> throw new IllegalStateException("Unexpected value: " + level);
        },
        target.getPose());
  }

  public static enum Locations {
    RS,
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
    L;
  }

  public static Command compileAuto(Locations... locations) {
    List<Command> commands = new ArrayList<>();
    for (int i = 0; i < locations.length - 1; i++) {
      try {
        var path = PathPlannerPath.fromChoreoTrajectory(locations[i] + "-" + locations[i + 1]);
        commands.add(AutoBuilder.followPath(path));
        commands.add(
            switch (locations[i + 1]) {
              case RHP -> DriveCommands.coralStationAlignRight(Subsystems.drive);
              case LHP -> DriveCommands.coralStationAlignLeft(Subsystems.drive);
              case A -> SuperstructureCommands.score(
                  ElevatorConstants.Setpoints.L4_PROPORTION,
                  FieldConstants.TargetPositions.REEF_A.getPose());
              case B -> SuperstructureCommands.score(
                  ElevatorConstants.Setpoints.L4_PROPORTION,
                  FieldConstants.TargetPositions.REEF_B.getPose());
              case C -> SuperstructureCommands.score(
                  ElevatorConstants.Setpoints.L4_PROPORTION,
                  FieldConstants.TargetPositions.REEF_C.getPose());
              case D -> SuperstructureCommands.score(
                  ElevatorConstants.Setpoints.L4_PROPORTION,
                  FieldConstants.TargetPositions.REEF_D.getPose());
              case E -> SuperstructureCommands.score(
                  ElevatorConstants.Setpoints.L4_PROPORTION,
                  FieldConstants.TargetPositions.REEF_E.getPose());
              case F -> SuperstructureCommands.score(
                  ElevatorConstants.Setpoints.L4_PROPORTION,
                  FieldConstants.TargetPositions.REEF_F.getPose());
              case G -> SuperstructureCommands.score(
                  ElevatorConstants.Setpoints.L4_PROPORTION,
                  FieldConstants.TargetPositions.REEF_G.getPose());
              case H -> SuperstructureCommands.score(
                  ElevatorConstants.Setpoints.L4_PROPORTION,
                  FieldConstants.TargetPositions.REEF_H.getPose());
              case I -> SuperstructureCommands.score(
                  ElevatorConstants.Setpoints.L4_PROPORTION,
                  FieldConstants.TargetPositions.REEF_I.getPose());
              case J -> SuperstructureCommands.score(
                  ElevatorConstants.Setpoints.L4_PROPORTION,
                  FieldConstants.TargetPositions.REEF_J.getPose());
              case K -> SuperstructureCommands.score(
                  ElevatorConstants.Setpoints.L4_PROPORTION,
                  FieldConstants.TargetPositions.REEF_K.getPose());
              case L -> SuperstructureCommands.score(
                  ElevatorConstants.Setpoints.L4_PROPORTION,
                  FieldConstants.TargetPositions.REEF_L.getPose());
              case RS -> throw new IllegalArgumentException("what the fuck");
            });
      } catch (IOException | ParseException e) {
        throw new RuntimeException(e);
      }
    }

    return Commands.sequence(commands.toArray(new Command[0]));
  }
}
