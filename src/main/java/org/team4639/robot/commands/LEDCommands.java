package org.team4639.robot.commands;

import edu.wpi.first.wpilibj.util.Color;
import edu.wpi.first.wpilibj2.command.Command;
import org.team4639.lib.led.pattern.CycleBetweenLEDPattern;
import org.team4639.lib.led.pattern.MovingWaveLEDPattern;
import org.team4639.lib.led.pattern.SolidLEDPattern;
import org.team4639.lib.led.pattern.TwoSegmentLEDPattern;
import org.team4639.lib.util.DriverStationUtil;
import org.team4639.robot.robot.Subsystems;

public class LEDCommands {
  public static Command disabled() {
    return Subsystems.leds
        .defer(
            () ->
                Subsystems.leds.usePattern(
                    new TwoSegmentLEDPattern(
                        new MovingWaveLEDPattern(
                            Color.kWhite,
                            DriverStationUtil.isBlueAlliance() ? Color.kFirstBlue : Color.kFirstRed,
                            3),
                        new MovingWaveLEDPattern(
                            Color.kWhite,
                            DriverStationUtil.isBlueAlliance() ? Color.kFirstBlue : Color.kFirstRed,
                            3),
                        0,
                        48,
                        48,
                        96,
                        true)))
        .ignoringDisable(true);
  }

  public static Command none() {
    return Subsystems.leds.usePattern(new SolidLEDPattern(Color.kBlack));
  }

  public static Command hasCoral() {
    return Subsystems.leds
        .usePattern(new CycleBetweenLEDPattern(7, Color.kGreen, Color.kBlack))
        .withTimeout(1)
        .andThen(none());
  }

  public static Command aligning() {
    return Subsystems.leds.usePattern(new SolidLEDPattern(Color.kGhostWhite));
  }
}
