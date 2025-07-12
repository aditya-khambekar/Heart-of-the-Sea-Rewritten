package org.team4639.robot.statemachine.states;

import org.team4639.lib.statebased.State;
import org.team4639.robot.constants.robot.Controls;
import org.team4639.robot.robot.Subsystems;

/**
 * Extends the State class to allow the driver to select the upcoming reef level
 * at any time without interfering with the dual binding of algae buttons during algae states.
 * <p></p>
 * A state being a CoralCycleState does not necessarily mean it can only be used as part of a coral cycle,
 * as is the case with IDLE and the homing states.
 */
public class CoralCycleState extends State {

    public CoralCycleState(String name) {
        super(name);
        super.and(Controls.L4_CORAL).onTrue(Subsystems.dashboardOutputs.setL4());
        super.and(Controls.L3_CORAL).onTrue(Subsystems.dashboardOutputs.setL3());
        super.and(Controls.L2_CORAL).onTrue(Subsystems.dashboardOutputs.setL2());
        super.and(Controls.L1_CORAL).onTrue(Subsystems.dashboardOutputs.setL1());
    }
}
