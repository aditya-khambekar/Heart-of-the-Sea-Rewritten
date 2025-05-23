package org.team4639.robot.modaltriggers;

import edu.wpi.first.wpilibj2.command.button.Trigger;
import org.team4639.robot.robot.RobotContainer;

public class IOTriggers {
  public static final Trigger hasDriverJoystickInput =
      new Trigger(
          () ->
              RobotContainer.driver.getLeftX() != 0
                  || RobotContainer.driver.getLeftY() != 0
                  || RobotContainer.driver.getRightX() != 0
                  || RobotContainer.driver.getRightY() != 0);

  public static final Trigger hasDriverRotationalInput =
      new Trigger(() -> RobotContainer.driver.getRightX() != 0);
}
