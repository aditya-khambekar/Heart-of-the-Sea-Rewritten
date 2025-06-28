package org.team4639.robot.auto;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.path.PathPlannerPath;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import java.io.IOException;
import java.util.ArrayList;
import java.util.List;
import org.json.simple.parser.ParseException;
import org.team4639.robot.commands.AutoCommands;
import org.team4639.robot.constants.FieldConstants;
import org.team4639.robot.robot.Subsystems;

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
        commands.add(AutoBuilder.followPath(path));
        commands.add(
            switch (locations[i + 1]) {
              case RHP -> AutoCommands.intakeRight();
              case LHP -> AutoCommands.intakeLeft();
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
              case ALGH -> Commands.none();
              case ALIJ -> Commands.none();
              case ALGSC1 -> Commands.none();
              case ALGSC2 -> Commands.none();
              case ALGSC3 -> Commands.none();
              case RS, LS, MS -> throw new IllegalArgumentException("what the fuck");
            });
      } catch (IOException | ParseException e) {
        System.err.println(locations[i] + "-" + locations[i + 1]);
        throw new RuntimeException(e);
      }
    }
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

  public static Command TEST_1MTR() {
    try {
      var path = PathPlannerPath.fromChoreoTrajectory("TEST-1MTR");
      return Commands.runOnce(
              () -> Subsystems.drive.setPose(path.getStartingHolonomicPose().orElse(new Pose2d())))
          .andThen(AutoBuilder.followPath(path));
    } catch (IOException | ParseException e) {
      throw new RuntimeException(e);
    }
  }
}
