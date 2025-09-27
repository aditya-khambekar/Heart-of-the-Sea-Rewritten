package org.team4639.robot.statemachine.reefscape.coral.intake;

import edu.wpi.first.math.filter.Debouncer;
import org.team4639.lib.statebased.State;
import org.team4639.robot.commands.SuperstructureCommands;
import org.team4639.robot.constants.Controls;
import org.team4639.robot.robot.Subsystems;
import org.team4639.robot.statemachine.States;
import org.team4639.robot.statemachine.reefscape.coral.CoralCycleState;
import org.team4639.robot.subsystems.superstructure.wrist.WristConstants;

import java.util.function.Supplier;

public class IntakeState extends CoralCycleState {
    private static Debouncer hasCoralDebouncer = new Debouncer(WristConstants.HAS_CORAL_DEBOUNCE_SECONDS, Debouncer.DebounceType.kRising);

    public IntakeState(String name, Supplier<State> nextState) {
        super(name);
        this.whileTrue(
                SuperstructureCommands.hpLower()
                        .until(Subsystems.wrist::hasCoral)
                        .andThen(SuperstructureCommands.hp()));
        this.withEndCondition(
                () -> hasCoralDebouncer.calculate(Subsystems.wrist.hasCoral()), nextState);
        this.onEmergency(() -> States.IDLE);
        this.onTrigger(Controls.ALIGN_LEFT, () -> States.FORCE_INTAKE_INTO_LEFT_ALIGN);
        this.onTrigger(Controls.ALIGN_RIGHT, () -> States.FORCE_INTAKE_INTO_RIGHT_ALIGN);
    }
}
