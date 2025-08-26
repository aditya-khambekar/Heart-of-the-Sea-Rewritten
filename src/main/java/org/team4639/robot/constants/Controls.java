package org.team4639.robot.constants;

import edu.wpi.first.wpilibj2.command.button.Trigger;
import org.team4639.robot.robot.RobotContainer;

public class Controls {
  /**
   * Press this to cancel all automation and return to stow/idle states, outtake game pieces when
   * not scoring, and return to stow/idle state after using manual controls
   *
   * <p>Also sometimes called "Safe State" trigger
   */
  public static final Trigger EMERGENCY = RobotContainer.driver.y();

  // Align commands start the process of scoring. If a scoring location has already been selected,
  // then scoring will be done automatically

  public static final Trigger ALIGN_LEFT = RobotContainer.driver.povLeft();
  public static final Trigger ALIGN_RIGHT = RobotContainer.driver.povRight();

  // Signifies to the code which level you want to score on, once you align then scoring will
  // be done automatically if one of these is pressed. These persist after scoring is done,
  // so you would be able to score multiple game pieces on the same level using only one button
  // press

  public static final Trigger L1_CORAL_SCORE = RobotContainer.driver.leftTrigger();
  public static final Trigger L2_CORAL_SCORE = RobotContainer.driver.leftBumper();
  public static final Trigger L3_CORAL_SCORE = RobotContainer.driver.rightBumper();
  public static final Trigger L4_CORAL_SCORE = RobotContainer.driver.rightTrigger();

  public static final Trigger ALGAE_INTAKE_AUTO = RobotContainer.driver.povUp();

  // Press these buttons once to get the superstructure to the right state, and once again to
  // outtake

  public static final Trigger ALGAE_PROCESSOR = RobotContainer.driver.leftTrigger();
  public static final Trigger ALGAE_BARGE = RobotContainer.driver.rightTrigger();

  public static final Trigger L1_CORAL_MANUAL = RobotContainer.operator.leftTrigger();
  public static final Trigger L2_CORAL_MANUAL = RobotContainer.operator.leftBumper();
  public static final Trigger L3_CORAL_MANUAL = RobotContainer.operator.rightBumper();
  public static final Trigger L4_CORAL_MANUAL = RobotContainer.operator.rightTrigger();

  public static final Trigger ALGAE_INTAKE_LOW = RobotContainer.operator.povDown();
  public static final Trigger ALGAE_INTAKE_HIGH = RobotContainer.operator.povUp();
  public static final Trigger ALGAE_PROCESSOR_MANUAL = RobotContainer.operator.povRight();
  public static final Trigger ALGAE_BARGE_MANUAL = RobotContainer.operator.povLeft();

  public static final Trigger ROLLER_TRIGGER = RobotContainer.operator.a();
  public static final Trigger INTAKE = RobotContainer.operator.b();
  public static final Trigger ALGAE_STOW = RobotContainer.operator.x();

  public static final Trigger FORCE_HOMING = RobotContainer.operator.rightStick();
  public static final Trigger FORCE_GYRO_RESET = RobotContainer.operator.leftStick();
}
