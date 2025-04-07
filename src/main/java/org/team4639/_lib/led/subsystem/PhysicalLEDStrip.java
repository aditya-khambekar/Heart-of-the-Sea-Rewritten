package org.team4639._lib.led.subsystem;

import edu.wpi.first.wpilibj.AddressableLED;
import edu.wpi.first.wpilibj.AddressableLEDBuffer;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.util.Color;
import edu.wpi.first.wpilibj.util.Color8Bit;
import java.util.Objects;
import org.team4639._lib.annotation.PackagePrivate;
import org.team4639._lib.led.pattern.LEDPattern;
import org.team4639._lib.subsystem.Updatable;

public class PhysicalLEDStrip extends LEDStrip implements Updatable {
  private final AddressableLED led;
  private final AddressableLEDBuffer buffer;
  private final int length;
  private double timeAtLastReset = Timer.getFPGATimestamp();

  private LEDPattern currentPattern = LEDPattern.BLANK;

  private static volatile PhysicalLEDStrip instance;

  @PackagePrivate
  static synchronized PhysicalLEDStrip getInstance() {
    return Objects.requireNonNullElseGet(instance, () -> instance = new PhysicalLEDStrip(9, 96));
  }

  private PhysicalLEDStrip(int port, int length) {
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
      Color color = currentPattern.get(i, Timer.getFPGATimestamp() - timeAtLastReset);
      buffer.setLED(i, new Color8Bit(color));
    }
    led.setData(buffer);
  }

  @Override
  public void doResetTime() {
    timeAtLastReset = Timer.getFPGATimestamp();
  }
}
