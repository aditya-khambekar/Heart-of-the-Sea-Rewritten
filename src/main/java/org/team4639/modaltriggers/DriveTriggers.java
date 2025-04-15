package org.team4639.modaltriggers;

import edu.wpi.first.wpilibj2.command.button.Trigger;
import org.team4639._robot.Subsystems;
import org.team4639.constants.FieldConstants;

public class DriveTriggers {
  public static Trigger closeToLeftStation =
      new Trigger(
          () ->
              Subsystems.drive
                      .getPose()
                      .getTranslation()
                      .getDistance(FieldConstants.CoralStation.leftCenterFace.getTranslation())
                  < 1.5);

  public static Trigger closeToRightStation =
      new Trigger(
          () ->
              Subsystems.drive
                      .getPose()
                      .getTranslation()
                      .getDistance(FieldConstants.CoralStation.rightCenterFace.getTranslation())
                  < 1.5);
}
