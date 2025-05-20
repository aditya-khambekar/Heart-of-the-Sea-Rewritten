package org.team4639.auto;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.path.PathPlannerPath;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Transform2d;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import java.io.IOException;
import java.util.ArrayList;
import java.util.List;
import org.json.simple.parser.ParseException;
import org.team4639._robot.Subsystems;
import org.team4639.auto.AutoState.eAutoState;
import org.team4639.commands.DriveCommands;
import org.team4639.commands.SuperstructureCommands;
import org.team4639.constants.FieldConstants;
import org.team4639.subsystems.elevator.ElevatorConstants;

public class AutoFactory {

  public enum Locations {
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
        commands.add(AutoState.setState(eAutoState.PATH));
        commands.add(AutoBuilder.followPath(path));
        commands.add(AutoState.setState(eAutoState.ALIGN));
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
              case ALGH -> SuperstructureCommands.intakeAlgae(
                  FieldConstants.TargetPositions.REEF_GH
                      .getPose()
                      .transformBy(new Transform2d(0.2, 0, Rotation2d.kZero)));
              case ALIJ -> SuperstructureCommands.intakeAlgae(
                  FieldConstants.TargetPositions.REEF_IJ
                      .getPose()
                      .transformBy(new Transform2d(0.2, 0, Rotation2d.kZero)));
              case ALGSC1 -> Subsystems.drive.defer(
                  () ->
                      DriveCommands.PIDtowithVelocityReset(
                          Subsystems.drive,
                          Subsystems.drive.getPose(),
                          FieldConstants.TargetPositions.BARGE_FARCAGE.getPose()));
              case ALGSC2 -> Subsystems.drive.defer(
                  () ->
                      DriveCommands.PIDtowithVelocityReset(
                          Subsystems.drive,
                          Subsystems.drive.getPose(),
                          FieldConstants.TargetPositions.BARGE_MIDDLECAGE.getPose()));
              case ALGSC3 -> Subsystems.drive.defer(
                  () ->
                      DriveCommands.PIDtowithVelocityReset(
                          Subsystems.drive,
                          Subsystems.drive.getPose(),
                          FieldConstants.TargetPositions.BARGE_CLOSECAGE.getPose()));
              case RS, LS, MS -> throw new IllegalArgumentException("what the fuck");
            });
      } catch (IOException | ParseException e) {
        System.err.println(locations[i] + "-" + locations[i + 1]);
        throw new RuntimeException(e);
      }
    }
    commands.add(AutoState.setState(eAutoState.NOT_IN_AUTO));
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

  public static Command MS_G_ALGH_ALGSC1_ALIJ_ALGSC2() {
    return compileAuto(
        Locations.MS,
        Locations.G,
        Locations.ALGH,
        Locations.ALGSC1,
        Locations.ALIJ,
        Locations.ALGSC2);
  }
}
