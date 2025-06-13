package org.team4639.robot.modaltriggers;

import edu.wpi.first.wpilibj2.command.button.Trigger;

public class SuperstructureTriggers {
  public static Trigger hasCoral =
      new Trigger(() -> false); // new Trigger(Subsystems.scoring::hasCoral);

  /** Triggers automatic motion to the intake setpoint. */
  public static Trigger intake = new Trigger(() -> false);
  // hasCoral.negate().and(DriveTriggers.closeToLeftStation.or(DriveTriggers.closeToRightStation));

  /** Triggers automatically raising the elevator to a halfway point in preparation for scoring. */
  public static Trigger raiseElevator = new Trigger(() -> false);
  // hasCoral.and(
  // (DriveTriggers.closeToLeftStation.or(DriveTriggers.closeToRightStation)).negate());
}
