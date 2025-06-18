package org.team4639.robot.statemachine;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import java.util.Arrays;
import java.util.HashMap;
import java.util.Map;
import java.util.function.BooleanSupplier;

public class State {
  private final String name;
  Trigger trigger;
  State onEmergency;
  State onAccelerationLimit;
  Map<BooleanSupplier, State> endConditions;

  public State(String name) {
    this.name = name;
    trigger = new Trigger(() -> StateMachine.getState() == this);
    onEmergency = this;
    onAccelerationLimit = this;
    endConditions = new HashMap<>();
  }

  public String getName() {
    return name;
  }

  public State whileTrue(Command... commands) {
    Arrays.stream(commands).forEach(trigger::whileTrue);
    return this;
  }

  public State onEmergency(State state) {
    onEmergency = state;
    return this;
  }

  public State onAccelerationLimit(State state) {
    onAccelerationLimit = state;
    return this;
  }

  public State withEndCondition(BooleanSupplier condition, State nextState) {
    endConditions.put(condition, nextState);
    return this;
  }

  public void evaluate() {
    endConditions.forEach(
        (condition, state) -> StateMachine.setState(condition.getAsBoolean() ? state : this));
  }
}
