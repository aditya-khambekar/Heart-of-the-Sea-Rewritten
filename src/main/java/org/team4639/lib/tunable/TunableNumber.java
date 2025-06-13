package org.team4639.lib.tunable;

import edu.wpi.first.util.sendable.Sendable;
import edu.wpi.first.util.sendable.SendableBuilder;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import java.util.function.DoubleConsumer;
import java.util.function.Supplier;
import org.team4639.lib.functional.DoubleConsumer2;
import org.team4639.lib.functional.DoubleSupplier2;

/**
 * Creates a number tunable through SmartDashboard. This is generally not recommended for use, as
 * most of its use cases (PID gains) are themselves instances of {@link Sendable} and are much
 * easier to tune using their {@link Sendable#initSendable(SendableBuilder)} methods.
 *
 * <p>This is equivalent to creating a {@link Sendable} containing just one number.
 */
public class TunableNumber implements Sendable, DoubleSupplier2, Supplier<Double> {
  private double value;
  private DoubleConsumer sendableConsumer =
      new DoubleConsumer2() {

        /**
         * Performs this operation on the given argument.
         *
         * @param value the input argument
         */
        @Override
        public void accept(double value) {}
      };

  @Override
  public void initSendable(SendableBuilder sendableBuilder) {
    sendableBuilder.addDoubleProperty("Value", () -> value, sendableConsumer);
  }

  public void addDoubleConsumer(DoubleConsumer consumer) {
    sendableConsumer = sendableConsumer.andThen(consumer);
  }

  public TunableNumber withDefaultValue(double defaultValue) {
    if (this.get() == null || this.get() == 0) this.value = defaultValue;
    return this;
  }

  /**
   * Sends this TunableNumber to SmartDashboard.
   *
   * @param name
   */
  public TunableNumber send(String name) {
    SmartDashboard.putData(name, this);
    return this;
  }

  /**
   * Gets a result.
   *
   * @return a result
   */
  @Override
  public double getAsDouble() {
    return value;
  }

  /**
   * Gets a result.
   *
   * @return a result
   */
  @Override
  public Double get() {
    return value;
  }
}
