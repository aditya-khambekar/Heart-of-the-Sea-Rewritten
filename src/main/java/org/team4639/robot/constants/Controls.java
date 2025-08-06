package org.team4639.robot.constants;

import edu.wpi.first.wpilibj2.command.button.Trigger;
import org.team4639.robot.robot.RobotContainer;

public class Controls {
  public static final Trigger intake = RobotContainer.driver.a();
  public static final Trigger secondIntake = RobotContainer.driver.a();
  public static final Trigger emergency = RobotContainer.driver.y();

  public static final Trigger alignLeft = RobotContainer.driver.povLeft();
  public static final Trigger alignRight = RobotContainer.driver.povRight();

  public static final Trigger resetGyro = RobotContainer.driver.leftStick();

  public static final Trigger L1CoralScore = RobotContainer.driver.leftTrigger();
  public static final Trigger L2CoralScore = RobotContainer.driver.leftBumper();
  public static final Trigger L3CoralScore = RobotContainer.driver.rightBumper();
  public static final Trigger L4CoralScore = RobotContainer.driver.rightTrigger();

  public static final Trigger outtake = RobotContainer.driver.a();

  public static final Trigger elevatorUp = RobotContainer.operator.povUp();
  public static final Trigger elevatorDown = RobotContainer.operator.povDown();

  public static final Trigger wristUp = RobotContainer.operator.povLeft();
  public static final Trigger wristDown = RobotContainer.operator.povRight();

  public static final Trigger REEF_A =
      RobotContainer.driver.povDown().and(RobotContainer.driver.leftBumper());
  public static final Trigger REEF_B =
      RobotContainer.driver.povDown().and(RobotContainer.driver.rightBumper());
  public static final Trigger REEF_C =
      RobotContainer.driver.povDownRight().and(RobotContainer.driver.leftBumper());
  public static final Trigger REEF_D =
      RobotContainer.driver.povDownRight().and(RobotContainer.driver.rightBumper());
  public static final Trigger REEF_E =
      RobotContainer.driver.povUpRight().and(RobotContainer.driver.leftBumper());
  public static final Trigger REEF_F =
      RobotContainer.driver.povUpRight().and(RobotContainer.driver.rightBumper());
  public static final Trigger REEF_G =
      RobotContainer.driver.povUp().and(RobotContainer.driver.leftBumper());
  public static final Trigger REEF_H =
      RobotContainer.driver.povUp().and(RobotContainer.driver.rightBumper());
  public static final Trigger REEF_I =
      RobotContainer.driver.povUpLeft().and(RobotContainer.driver.leftBumper());
  public static final Trigger REEF_J =
      RobotContainer.driver.povUpLeft().and(RobotContainer.driver.rightBumper());
  public static final Trigger REEF_K =
      RobotContainer.driver.povDownLeft().and(RobotContainer.driver.leftBumper());
  public static final Trigger REEF_L =
      RobotContainer.driver.povDownLeft().and(RobotContainer.driver.rightBumper());

  public static final Trigger REEF_AB = RobotContainer.driver.povDown();
  public static final Trigger REEF_CD = RobotContainer.driver.povDownRight();
  public static final Trigger REEF_EF = RobotContainer.driver.povUpRight();
  public static final Trigger REEF_GH = RobotContainer.driver.povUp();
  public static final Trigger REEF_IJ = RobotContainer.driver.povUpLeft();
  public static final Trigger REEF_KL = RobotContainer.driver.povDownLeft();

  public static final Trigger LEFT_HP = RobotContainer.driver.leftBumper();
  public static final Trigger RIGHT_HP = RobotContainer.driver.rightBumper();
}
