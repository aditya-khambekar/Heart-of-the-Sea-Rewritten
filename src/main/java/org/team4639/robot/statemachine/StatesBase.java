package org.team4639.robot.statemachine;

import org.team4639.lib.statebased.State;

public interface StatesBase {
  /** Initializes the state machine */
  public void init();

  /** Gets the state at the start of auto */
  public State getAutoStartState();

  /** Gets the state at the start of teleop */
  public State getTeleopStartState();

  /**
   * Binds buttons that will be used when this state machine is running. When it is not running,
   * the buttons will not be bound.
   */
  public void initButtonBindings();
}
