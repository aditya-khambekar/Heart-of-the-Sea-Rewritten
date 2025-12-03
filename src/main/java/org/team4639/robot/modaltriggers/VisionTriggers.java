package org.team4639.robot.modaltriggers;

import edu.wpi.first.wpilibj2.command.button.Trigger;
import org.team4639.robot.subsystems.vision.BaseVisionConsumer;

public class VisionTriggers {
  private static Trigger visionIsActive =
      new Trigger(BaseVisionConsumer.getInstance()::isVisionActive);

  public static Trigger visionIsActive() {
    return visionIsActive;
  }
}
