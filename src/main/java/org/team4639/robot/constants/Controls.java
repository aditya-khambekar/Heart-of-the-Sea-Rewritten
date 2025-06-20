package org.team4639.robot.constants;

import edu.wpi.first.wpilibj2.command.button.Trigger;
import org.team4639.robot.robot.RobotContainer;

public class Controls {
  public static final Trigger intake = RobotContainer.driver.a();
  public static final Trigger secondIntake = RobotContainer.driver.b();
  public static final Trigger emergency = RobotContainer.operator.y();

  public static final Trigger alignLeft = RobotContainer.driver.leftBumper();
  public static final Trigger alignRight = RobotContainer.driver.rightBumper();

  public static final Trigger L1Coral = RobotContainer.operator.leftTrigger();
  public static final Trigger L2Coral = RobotContainer.operator.leftBumper();
  public static final Trigger L3Coral = RobotContainer.operator.rightBumper();
  public static final Trigger L4Coral = RobotContainer.operator.rightTrigger();
}
