package org.team4639.robot.commands.superstructure;

public enum SuperstructureCommandState {
  TO_SAFE_ANGLE,
  TO_ELEVATOR_SETPOINT,
  TO_WRIST_SETPOINT,
  EXECUTING_ACTION,
  DONE,
  HOMING,
  MICROADJUSTMENTS,
  STOPPED;
}
