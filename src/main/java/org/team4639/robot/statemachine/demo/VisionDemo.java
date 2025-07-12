package org.team4639.robot.statemachine.demo;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Transform2d;
import java.util.Objects;
import org.team4639.lib.statebased.State;
import org.team4639.lib.statebased.StateFactory;
import org.team4639.lib.unit.Units2;
import org.team4639.robot.commands.DriveCommands;
import org.team4639.robot.commands.SuperstructureCommands;
import org.team4639.robot.constants.reefscape.TargetPositions;
import org.team4639.robot.constants.robot.Controls;
import org.team4639.robot.statemachine.StatesBase;

public final class VisionDemo implements StatesBase {
  public static State DEMO_ON;
  public static State DEMO_OFF;

  private static volatile StatesBase instance;

  public static synchronized StatesBase getInstance() {
    return instance = Objects.requireNonNullElseGet(instance, VisionDemo::new);
  }

  private VisionDemo() {}

  public void init() {
    DEMO_ON =
        new State("DEMO_ON")
            .whileTrue(
                SuperstructureCommands.IDLE,
                DriveCommands.Evergreen.PIDToUnending(
                    TargetPositions.REEF_AB
                        .getPose()
                        .transformBy(
                            new Transform2d(
                                Units2.inchesToMeters.convert(-36), 0, Rotation2d.kZero))))
            .withEndCondition(Controls.VisionDemoControls.DEMO_ON.negate(), () -> DEMO_OFF);
    DEMO_OFF =
        new State("DEMO_OFF")
            .whileTrue(SuperstructureCommands.IDLE)
            .onTrigger(Controls.VisionDemoControls.DEMO_ON, () -> DEMO_ON);
  }

  /** Gets the state at the start of auto */
  @Override
  public State getAutoStartState() {
    return StateFactory.none();
  }

  /** Gets the state at the start of teleop */
  @Override
  public State getTeleopStartState() {
    return DEMO_OFF;
  }
}
