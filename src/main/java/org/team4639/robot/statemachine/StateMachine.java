package org.team4639.robot.statemachine;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import java.util.Objects;

public class StateMachine extends SubsystemBase {
  private static volatile StateMachine instance;

  public static synchronized StateMachine getInstance() {
    return instance = Objects.requireNonNullElseGet(instance, StateMachine::new);
  }

  private State state;

  public static State getState() {
    return getInstance().state;
  }

  public static void setState(State newState) {
    if (newState != getInstance().state) getInstance().state = newState;
  }

  @Override
  public void periodic() {
    state.evaluate();
  }
}
