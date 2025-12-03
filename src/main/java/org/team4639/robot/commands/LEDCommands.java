package org.team4639.robot.commands;

import edu.wpi.first.wpilibj.util.Color;
import edu.wpi.first.wpilibj2.command.Command;
import org.team4639.lib.led.pattern.CycleBetweenLEDPattern;
import org.team4639.lib.led.pattern.MovingLEDPattern;
import org.team4639.lib.led.pattern.SolidLEDPattern;
import org.team4639.lib.led.pattern.TwoSegmentLEDPattern;
import org.team4639.robot.robot.Subsystems;

public class LEDCommands {
  public static Command disabled() {
    /*return Subsystems.leds
    .defer(
        () ->
            Subsystems.leds.usePattern(
                new TwoSegmentLEDPattern(
                    new MovingLEDPattern(Color.kBlack, new Color(255, 125, 125), -3, 0.2, 1),
                    new MovingLEDPattern(Color.kBlack, new Color(255, 125, 125), -3, 0.2, 1),
                    4,
                    32,
                    65,
                    94,
                    true)))
    .ignoringDisable(true);*/

    return Subsystems.leds
        .defer(
            () ->
                Subsystems.leds.usePattern(
                    new TwoSegmentLEDPattern(
                        new MovingLEDPattern(Color.kBlack, Color.kMaroon, -3, 0.2, 1),
                        new MovingLEDPattern(Color.kBlack, Color.kMaroon, -3, 0.2, 1),
                        4,
                        32,
                        65,
                        94,
                        true)))
        .ignoringDisable(true);
  }

  public static Command enabledDefault() {
    return Subsystems.leds
        .defer(
            () ->
                Subsystems.leds.usePattern(
                    new TwoSegmentLEDPattern(
                        new MovingLEDPattern(Color.kBlack, new Color(255, 35, 0), -3, 0.2, 1),
                        new MovingLEDPattern(Color.kBlack, new Color(255, 35, 0), -3, 0.2, 1),
                        4,
                        32,
                        65,
                        94,
                        true)))
        .ignoringDisable(true);
  }

  public static Command manual() {
    return Subsystems.leds
        .defer(
            () ->
                Subsystems.leds.usePattern(
                    new TwoSegmentLEDPattern(
                        new MovingLEDPattern(Color.kBlack, new Color(255, 25, 50), -3, 0.2, 1),
                        new MovingLEDPattern(Color.kBlack, new Color(255, 25, 50), -3, 0.2, 1),
                        4,
                        32,
                        65,
                        94,
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
