package org.team4639.commands.drive;

import edu.wpi.first.wpilibj2.command.button.Trigger;
import constants.FieldConstants;
import _robot.Subsystems;

public class DriveTriggers {
  public static Trigger closeToLeftStation =
      new Trigger(
          () ->
              Subsystems.drive
                      .getPose()
                      .getTranslation()
                      .getDistance(FieldConstants.CoralStation.leftCenterFace.getTranslation())
                  < 2.5);

  public static Trigger closeToRightStation =
      new Trigger(
          () ->
              Subsystems.drive
                      .getPose()
                      .getTranslation()
                      .getDistance(FieldConstants.CoralStation.rightCenterFace.getTranslation())
                  < 2.5);
}
