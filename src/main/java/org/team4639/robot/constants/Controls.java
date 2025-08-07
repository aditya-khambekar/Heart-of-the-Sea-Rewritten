package org.team4639.robot.constants;

import edu.wpi.first.wpilibj2.command.button.Trigger;
import org.team4639.robot.robot.RobotContainer;

public class Controls {
  public static final Trigger EMERGENCY = RobotContainer.driver.y();

  public static final Trigger ALIGN_LEFT = RobotContainer.driver.povLeft();
  public static final Trigger ALIGN_RIGHT = RobotContainer.driver.povRight();

  public static final Trigger L1_CORAL_SCORE = RobotContainer.driver.leftTrigger();
  public static final Trigger L2_CORAL_SCORE = RobotContainer.driver.leftBumper();
  public static final Trigger L3_CORAL_SCORE = RobotContainer.driver.rightBumper();
  public static final Trigger L4_CORAL_SCORE = RobotContainer.driver.rightTrigger();

  public static final Trigger ALGAE_INTAKE_AUTO = RobotContainer.driver.povUp();
  public static final Trigger ALGAE_PROCESSOR = RobotContainer.driver.leftTrigger();
  public static final Trigger ALGAE_BARGE = RobotContainer.driver.rightTrigger();

  public static final Trigger L1_CORAL = RobotContainer.operator.leftTrigger();
  public static final Trigger L2_CORAL = RobotContainer.operator.leftBumper();
  public static final Trigger L3_CORAL = RobotContainer.operator.rightBumper();
  public static final Trigger L4_CORAL = RobotContainer.operator.rightTrigger();

  public static final Trigger ALGAE_INTAKE_LOW = RobotContainer.operator.povDown();
  public static final Trigger ALGAE_INTAKE_HIGH = RobotContainer.operator.povUp();
  public static final Trigger ALGAE_PROCESSOR_MANUAL = RobotContainer.operator.povRight();
  public static final Trigger ALGAE_BARGE_MANUAL = RobotContainer.operator.povLeft();
}
