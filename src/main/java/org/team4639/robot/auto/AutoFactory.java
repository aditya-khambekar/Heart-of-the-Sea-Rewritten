package org.team4639.robot.auto;

import static org.team4639.robot.auto.AutoGenerator.Location.*;
import static org.team4639.robot.auto.AutoGenerator.compileAuto;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.path.PathPlannerPath;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import java.io.IOException;
import org.json.simple.parser.ParseException;
import org.team4639.robot.robot.Subsystems;

public final class AutoFactory {
  public static Command RS_F4_E4_D4_C4() {
    return compileAuto(RS, F4, RHP, E4, RHP, D4, RHP, C4);
  }

  public static Command RS_F4_E4_D4() {
    return compileAuto(RS, F4, RHP, E4, RHP, D4);
  }

  public static Command RS_E4_D4_C4() {
    return compileAuto(RS, E4, RHP, D4, RHP, C4);
  }

  public static Command RS_F4_D4_C4_E4() {
    return compileAuto(RS, F4, RHP, D4, RHP, C4, RHP, E4);
  }

  public static Command LS_I4_J4_K4_L4() {
    return compileAuto(LS, I4, LHP, J4, LHP, K4, LHP, L4);
  }

  public static Command LS_I4_J4_K4() {
    return compileAuto(LS, I4, LHP, J4, LHP, K4);
  }

  public static Command LS_J4_K4_L4() {
    return compileAuto(LS, J4, LHP, K4, LHP, L4);
  }

  public static Command LS_I4_K4_L4_J4() {
    return compileAuto(LS, I4, LHP, K4, LHP, L4, LHP, J4);
  }

  public static Command MS_G4_ALGH_ALGSC1_ALIJ_ALGSC2() {
    return compileAuto(MS, G4, ALGH, ALGSC1, ALIJ, ALGSC2);
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
