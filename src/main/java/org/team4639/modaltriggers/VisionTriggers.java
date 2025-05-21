package org.team4639.modaltriggers;

import org.team4639._robot.Subsystems;
import org.team4639.subsystems.vision.VisionUpdates;

import edu.wpi.first.wpilibj2.command.button.Trigger;

public class VisionTriggers {
    private static Trigger visionIsActive = new Trigger(VisionUpdates.getInstance()::isVisionActive);

    public static Trigger visionIsActive() {
        return visionIsActive;
    }
}
