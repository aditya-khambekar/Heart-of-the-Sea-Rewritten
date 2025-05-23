package org.team4639.modaltriggers;

import edu.wpi.first.wpilibj2.command.button.Trigger;
import org.team4639.subsystems.vision.VisionUpdates;

public class VisionTriggers {
  private static Trigger visionIsActive = new Trigger(VisionUpdates.getInstance()::isVisionActive);

  public static Trigger visionIsActive() {
    return visionIsActive;
  }
}
