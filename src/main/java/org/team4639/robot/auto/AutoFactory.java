package org.team4639.robot.auto;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.path.PathPlannerPath;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import java.io.IOException;
import org.json.simple.parser.ParseException;
import org.team4639.robot.commands.DriveCommands;
import org.team4639.robot.robot.Subsystems;

public class AutoFactory {
  public static Command RS_F_E_D_C() {
    return AutoGenerator.compileAuto(
        AutoGenerator.Location.RS,
        AutoGenerator.Location.F,
        AutoGenerator.Location.RHP,
        AutoGenerator.Location.E,
        AutoGenerator.Location.RHP,
        AutoGenerator.Location.D,
        AutoGenerator.Location.RHP,
        AutoGenerator.Location.C);
  }

  public static Command RS_F_E_D() {
    return AutoGenerator.compileAuto(
        AutoGenerator.Location.RS,
        AutoGenerator.Location.F,
        AutoGenerator.Location.RHP,
        AutoGenerator.Location.E,
        AutoGenerator.Location.RHP,
        AutoGenerator.Location.D);
  }

  public static Command RS_E_D_C() {
    return AutoGenerator.compileAuto(
        AutoGenerator.Location.RS,
        AutoGenerator.Location.E,
        AutoGenerator.Location.RHP,
        AutoGenerator.Location.D,
        AutoGenerator.Location.RHP,
        AutoGenerator.Location.C);
  }

  public static Command LS_I_J_K_L() {
    return AutoGenerator.compileAuto(
        AutoGenerator.Location.LS,
        AutoGenerator.Location.I,
        AutoGenerator.Location.LHP,
        AutoGenerator.Location.J,
        AutoGenerator.Location.LHP,
        AutoGenerator.Location.K,
        AutoGenerator.Location.LHP,
        AutoGenerator.Location.L);
  }

  public static Command LS_I_J_K() {
    return AutoGenerator.compileAuto(
        AutoGenerator.Location.LS,
        AutoGenerator.Location.I,
        AutoGenerator.Location.LHP,
        AutoGenerator.Location.J,
        AutoGenerator.Location.LHP,
        AutoGenerator.Location.K);
  }

  public static Command LS_J_K_L() {
    return AutoGenerator.compileAuto(
        AutoGenerator.Location.LS,
        AutoGenerator.Location.J,
        AutoGenerator.Location.LHP,
        AutoGenerator.Location.K,
        AutoGenerator.Location.LHP,
        AutoGenerator.Location.L);
  }

  public static Command MS_G_ALGH_ALGSC1_ALIJ_ALGSC2() {
    return AutoGenerator.compileAuto(
        AutoGenerator.Location.MS,
        AutoGenerator.Location.G,
        AutoGenerator.Location.ALGH,
        AutoGenerator.Location.ALGSC1,
        AutoGenerator.Location.ALIJ,
        AutoGenerator.Location.ALGSC2);
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

  public static Command leave() {

    return DriveCommands.joystickDrive(() -> -0.4, () -> 0, () -> 0).withTimeout(3);
  }
}
