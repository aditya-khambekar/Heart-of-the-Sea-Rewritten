package org.team4639.robot.modaltriggers;

import edu.wpi.first.wpilibj2.command.button.Trigger;
import org.team4639.robot.robot.Subsystems;

public class SuperstructureTriggers {
  public static Trigger hasCoral = new Trigger(Subsystems.scoring::hasCoral);

  /** Triggers automatic motion to the intake setpoint. */
  public static Trigger intake =
      hasCoral.negate().and(DriveTriggers.closeToLeftStation.or(DriveTriggers.closeToRightStation));

  /** Triggers automatically raising the elevator to a halfway point in preparation for scoring. */
  public static Trigger raiseElevator =
      hasCoral.and(
          (DriveTriggers.closeToLeftStation.or(DriveTriggers.closeToRightStation)).negate());
}
