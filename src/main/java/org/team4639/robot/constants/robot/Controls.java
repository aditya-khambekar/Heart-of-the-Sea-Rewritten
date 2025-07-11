package org.team4639.robot.constants.robot;

import static org.team4639.robot.robot.RobotContainer.driver;
import static org.team4639.robot.robot.RobotContainer.operator;

import org.team4639.lib.oi.RSTrigger;
import org.team4639.robot.constants.reefscape.FieldPoseUtil;
import org.team4639.robot.statemachine.competition.ReefscapeStates;
import org.team4639.robot.statemachine.demo.VisionDemo;

public final class Controls {
  public static final RSTrigger EMERGENCY = driver.y();

  public static final RSTrigger ALIGN_LEFT = driver.povLeft();
  public static final RSTrigger ALIGN_RIGHT = driver.povRight();

  public static final RSTrigger RESET_GYRO = driver.ab();

  public static final RSTrigger L1_CORAL = driver.leftTrigger().or(operator.leftTrigger());
  public static final RSTrigger L2_CORAL = driver.leftBumper().or(operator.leftBumper());
  public static final RSTrigger L3_CORAL = driver.rightBumper().or(operator.leftBumper());
  public static final RSTrigger L4_CORAL = driver.rightTrigger().or(operator.rightTrigger());

  public static final RSTrigger PROCESSOR = driver.leftTrigger().or(operator.leftTrigger());
  public static final RSTrigger ALGAE_INTAKE =
      driver
          .leftBumper()
          .or(operator.leftBumper())
          .or(driver.rightBumper())
          .or(operator.leftBumper());

  public static final RSTrigger BARGE_SCORE = driver.rightTrigger().or(operator.rightTrigger());

  public static final RSTrigger MICRO_ADJUSTMENTS_OUTTAKE =
      ReefscapeStates.getInstance().MICROADJUSTMENTS.and(operator.a());

  // TODO: figure out what to do with these
  // Microadjustment triggers
  public static final RSTrigger ELEVATOR_UP = operator.povUp();
  public static final RSTrigger ELEVATOR_DOWN = operator.povDown();

  public static final RSTrigger WRIST_UP = operator.povLeft();
  public static final RSTrigger WRIST_DOWN = operator.povRight();

  private static final RSTrigger HP_ALIGN = driver.x();

  public static final RSTrigger LEFT_HP = HP_ALIGN.and(FieldPoseUtil::closerToLeftStation);
  public static final RSTrigger RIGHT_HP = HP_ALIGN.andNot(FieldPoseUtil::closerToLeftStation);

  public static final RSTrigger DEFENSE_TOGGLE = driver.rightStick();

  public static final class VisionDemoControls {
    public static final RSTrigger DEMO_ON =
        driver.x().and(VisionDemo.DEMO_ON.getTrigger().or(VisionDemo.DEMO_OFF.getTrigger()));
  }
}
