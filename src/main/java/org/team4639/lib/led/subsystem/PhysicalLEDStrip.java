package org.team4639.lib.led.subsystem;

import edu.wpi.first.wpilibj.AddressableLED;
import edu.wpi.first.wpilibj.AddressableLEDBuffer;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.util.Color;
import edu.wpi.first.wpilibj.util.Color8Bit;
import org.team4639.lib.led.pattern.LEDPattern;

public class PhysicalLEDStrip extends LEDStrip {
  private final AddressableLED led;
  private final AddressableLEDBuffer buffer;
  private final int length;

  private LEDPattern currentPattern = LEDPattern.BLANK;

  public PhysicalLEDStrip(int port, int length) {
    this.length = length;

    led = new AddressableLED(port);
    buffer = new AddressableLEDBuffer(length);

    led.setLength(length);
    led.setData(buffer);
    led.start();
  }

  @Override
  public void setPattern(LEDPattern pattern) {
    currentPattern = pattern;
  }

  @Override
  public void update() {
    for (int i = 0; i < length; i++) {
      Color color = currentPattern.get(i, Timer.getFPGATimestamp());
      buffer.setLED(i, new Color8Bit(color));
    }
    led.setData(buffer);
  }
}
