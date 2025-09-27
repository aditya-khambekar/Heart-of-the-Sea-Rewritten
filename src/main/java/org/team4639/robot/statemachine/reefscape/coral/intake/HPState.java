package org.team4639.robot.statemachine.reefscape.coral.intake;

import org.team4639.robot.constants.Controls;
import org.team4639.robot.robot.RobotContainer;
import org.team4639.robot.statemachine.States;
import org.team4639.robot.statemachine.reefscape.coral.CoralCycleState;

public class HPState extends CoralCycleState {
    public HPState(String name) {
        super(name);
        this.onTrigger(RobotContainer.driver.a(), () -> States.INTAKE_LOWER);
        this.onTrigger(Controls.ALIGN_LEFT, () -> States.INTAKE_LOWER_INTO_LEFT_ALIGN);
        this.onTrigger(Controls.ALIGN_RIGHT, () -> States.INTAKE_LOWER_INTO_RIGHT_ALIGN);
        this.onEmergency(() -> States.IDLE);
    }
}
