package org.team4639.robot.statemachine.reefscape.coral;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import org.team4639.robot.constants.Controls;
import org.team4639.robot.statemachine.reefscape.ReefscapeState;
import org.team4639.robot.subsystems.DashboardOutputs;

import java.util.Map;

public class CoralCycleState extends ReefscapeState {
    private static final Map<Trigger, Command> driverInputToDashboardOutputSelector =
            Map.ofEntries(
                    Map.entry(Controls.L4_CORAL_SCORE, DashboardOutputs.getInstance().selectL4()),
                    Map.entry(Controls.L3_CORAL_SCORE, DashboardOutputs.getInstance().selectL3()),
                    Map.entry(Controls.L2_CORAL_SCORE, DashboardOutputs.getInstance().selectL2()),
                    Map.entry(Controls.L1_CORAL_SCORE, DashboardOutputs.getInstance().selectL1()));

    public CoralCycleState(String name) {
        super(name);
        this.mapTriggerCommandsOnTrue(driverInputToDashboardOutputSelector);
    }
}
