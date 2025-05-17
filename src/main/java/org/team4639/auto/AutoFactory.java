package org.team4639.auto;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.path.PathPlannerPath;
import edu.wpi.first.math.geometry.Pose2d;
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

  public static enum Locations {
    RS,
    LS,
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
    List<PathPlannerPath> paths = new ArrayList<>();
    for (int i = 0; i < locations.length - 1; i++) {
      try {
        var path = PathPlannerPath.fromChoreoTrajectory(locations[i] + "-" + locations[i + 1]);
        paths.add(path);
        if (i == 0)
          commands.add(
              Commands.runOnce(
                  () ->
                      Subsystems.drive.setPose(
                          path.getStartingHolonomicPose().orElse(new Pose2d()))));
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
              case RS, LS -> throw new IllegalArgumentException("what the fuck");
            });
      } catch (IOException | ParseException e) {
        System.err.println(locations[i] + "-" + locations[i + 1]);
        throw new RuntimeException(e);
      }
    }
    //    commands.add(
    //        0,
    //        Commands.runOnce(
    //            () -> {
    //              for (PathPlannerPath path : paths) {
    //                Subsystems.drive
    //                    .getField()
    //                    .getObject(path.toString())
    //                    .setPoses(path.getPathPoses());
    //              }
    //            }));
       return Commands.sequence(commands.toArray(new Command[0]));
  }

  public static Command RS_F_E_D_C() {
    return compileAuto(
        Locations.RS,
        Locations.F,
        Locations.RHP,
        Locations.E,
        Locations.RHP,
        Locations.D,
        Locations.RHP,
        Locations.C);
  }

  public static Command RS_F_E_D() {
    return compileAuto(
        Locations.RS, Locations.F, Locations.RHP, Locations.E, Locations.RHP, Locations.D);
  }

  public static Command RS_E_D_C() {
    return compileAuto(
        Locations.RS, Locations.E, Locations.RHP, Locations.D, Locations.RHP, Locations.C);
  }

  public static Command LS_I_J_K_L() {
    return compileAuto(
        Locations.LS,
        Locations.I,
        Locations.LHP,
        Locations.J,
        Locations.LHP,
        Locations.K,
        Locations.LHP,
        Locations.L);
  }

  public static Command LS_I_J_K() {
    return compileAuto(
        Locations.LS, Locations.I, Locations.LHP, Locations.J, Locations.LHP, Locations.K);
  }

  public static Command LS_J_K_L() {
    return compileAuto(
        Locations.LS, Locations.J, Locations.LHP, Locations.K, Locations.LHP, Locations.L);
  }
}
