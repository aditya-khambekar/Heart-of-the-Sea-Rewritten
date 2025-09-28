package org.team4639.robot.statemachine.reefscape.coral.outtake;

import org.team4639.robot.robot.Subsystems;
import org.team4639.robot.statemachine.States;
import org.team4639.robot.statemachine.reefscape.coral.CoralCycleState;

public class CoralScoreState extends CoralCycleState {
  public CoralScoreState(String name) {
    super(name);
    this.withEndCondition(Subsystems.wrist::doesNotHaveCoral, () -> States.HOMING_READY);
    this.onEmergency(() -> States.HOMING_READY);
  }
}
