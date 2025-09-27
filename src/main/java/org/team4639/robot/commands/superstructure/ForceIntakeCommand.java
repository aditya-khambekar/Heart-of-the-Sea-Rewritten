package org.team4639.robot.commands.superstructure;

import edu.wpi.first.math.filter.Debouncer;
import org.team4639.robot.robot.Subsystems;
import org.team4639.robot.subsystems.superstructure.SuperstructureSetpoints;
import org.team4639.robot.subsystems.superstructure.wrist.WristConstants;

import static edu.wpi.first.units.Units.Value;

public class ForceIntakeCommand extends SuperstructureCommandBase {
    private static final Debouncer hasCoralDebouncer = new Debouncer(WristConstants.HAS_CORAL_DEBOUNCE_SECONDS, Debouncer.DebounceType.kRising);

    public ForceIntakeCommand() {
        super();
        addRequirements(
                Subsystems.elevator, Subsystems.wrist, Subsystems.roller, Subsystems.superstructure);
    }

    @Override
    public SuperstructureCommandState getState() {
        return SuperstructureCommandState.EXECUTING_ACTION;
    }

    @Override
    public String getName() {
        return "Force Intake";
    }

    @Override
    public void initialize() {
        Subsystems.elevator.elevatorStop();
        Subsystems.wrist.setWristDutyCycle(Value.zero());
        Subsystems.roller.setDutyCycle(Value.zero());
    }

    @Override
    public void execute() {
        super.execute();
        Subsystems.elevator.elevatorStop();
        Subsystems.wrist.setWristDutyCycle(Value.zero());

        Subsystems.roller.setVelocity(SuperstructureSetpoints.HP_LOWER.wheelSpeed());
    }

    @Override
    public void end(boolean interrupted) {
        Subsystems.elevator.elevatorStop();
        Subsystems.wrist.setWristDutyCycle(Value.zero());
        Subsystems.roller.setDutyCycle(Value.zero());
    }

    @Override
    public boolean isFinished() {
        return hasCoralDebouncer.calculate(Subsystems.wrist.hasCoral());
    }
}
