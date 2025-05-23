package org.team4639.robot.auto;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;

public class AutoState {
  public enum eAutoState {
    PATH,
    ALIGN,
    NOT_IN_AUTO
  }

  private static eAutoState state = eAutoState.NOT_IN_AUTO;

  public static Command setState(eAutoState newState) {
    return Commands.runOnce(() -> AutoState.state = newState);
  }

  public eAutoState getState() {
    return state;
  }
}
