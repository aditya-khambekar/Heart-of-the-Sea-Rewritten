package org.team4639.lib.oi;

import edu.wpi.first.wpilibj.event.EventLoop;
import edu.wpi.first.wpilibj2.command.button.Trigger;

import java.util.HashMap;
import java.util.Map;
import java.util.Objects;
import java.util.function.BooleanSupplier;

/**
 * Adds more syntactically desirable methods and caching to the Trigger class.
 */
public class RSTrigger extends Trigger {
  private final Map<BooleanSupplier, RSTrigger> andCache = new HashMap<>();
  private final Map<BooleanSupplier, RSTrigger> orCache = new HashMap<>();
  private RSTrigger negated = null;

  /**
   * Creates a new trigger based on the given condition.
   *
   * @param loop The loop instance that polls this trigger.
   * @param condition the condition represented by this trigger
   */
  public RSTrigger(EventLoop loop, BooleanSupplier condition) {
    super(loop, condition);
  }

  /**
   * Creates a new trigger based on the given condition.
   *
   * <p>Polled by the default scheduler button loop.
   *
   * @param condition the condition represented by this trigger
   */
  public RSTrigger(BooleanSupplier condition) {
    super(condition);
  }

  public RSTrigger andNot(BooleanSupplier trigger) {
    return this.and(() -> !trigger.getAsBoolean());
  }

  public RSTrigger orNot(BooleanSupplier trigger) {
    return this.or(() -> !trigger.getAsBoolean());
  }

  public static RSTrigger of(BooleanSupplier condition) {
    return new RSTrigger(condition);
  }

  @Override
  public RSTrigger and(BooleanSupplier trigger) {
    return andCache.computeIfAbsent(trigger, t -> new RSTrigger(() -> super.getAsBoolean() && t.getAsBoolean()));
  }

  @Override
  public RSTrigger or(BooleanSupplier trigger) {
    return orCache.computeIfAbsent(trigger, t -> new RSTrigger(() -> super.getAsBoolean() || t.getAsBoolean()));
  }

  @Override
  public RSTrigger negate() {
    return negated = Objects.requireNonNullElseGet(negated, () -> new RSTrigger(() -> !super.getAsBoolean()));
  }
}
