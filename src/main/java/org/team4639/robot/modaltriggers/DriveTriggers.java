package org.team4639.robot.modaltriggers;

import edu.wpi.first.wpilibj2.command.button.Trigger;
import org.team4639.robot.constants.FieldConstants;
import org.team4639.robot.robot.Subsystems;

public class DriveTriggers {
  public static Trigger closeToLeftStation =
      new Trigger(
              () ->
                  Subsystems.drive
                          .getPose()
                          .getTranslation()
                          .getDistance(FieldConstants.CoralStation.leftCenterFace.getTranslation())
                      < .75)
          .and(VisionTriggers.visionIsActive());

  public static Trigger closeToRightStation =
      new Trigger(
              () ->
                  Subsystems.drive
                          .getPose()
                          .getTranslation()
                          .getDistance(FieldConstants.CoralStation.rightCenterFace.getTranslation())
                      < .75)
          .and(VisionTriggers.visionIsActive());
}
