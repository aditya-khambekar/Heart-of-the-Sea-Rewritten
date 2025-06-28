package org.team4639.robot.commands.superstructure;

import static edu.wpi.first.units.Units.Percent;
import static edu.wpi.first.units.Units.RotationsPerSecond;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.wpilibj2.command.Command;
import java.util.function.BooleanSupplier;
import org.team4639.robot.robot.Subsystems;
import org.team4639.robot.subsystems.superstructure.Superstructure;
import org.team4639.robot.subsystems.superstructure.SuperstructureState;
import org.team4639.robot.subsystems.superstructure.elevator.ElevatorConstants;

public class SuperstructureCommand extends Command {
  private SuperstructureState setpoint;
  private SuperstructureCommandState state;
  private BooleanSupplier endCondition;

  /**
   * Commands the superstructure to go to a specific state
   *
   * @param setpoint the setpoint that the superstructure is commanded to
   * @param endCondition the condition to end this command. Ex. has coral, doesn't have coral, or
   *     can always return false for a command that doesn't end by itself. This is checked only once
   *     the command has reached the EXECUTING_ACTION state.
   */
  public SuperstructureCommand(SuperstructureState setpoint, BooleanSupplier endCondition) {
    addRequirements(
        Subsystems.elevator, Subsystems.wrist, Subsystems.roller, Subsystems.superstructure);
    this.setpoint = setpoint;
    this.state = SuperstructureCommandState.TO_SAFE_ANGLE;
    this.endCondition = endCondition;
  }

  public SuperstructureCommand(SuperstructureState setpoint) {
    this(setpoint, () -> false);
  }

  @Override
  public void initialize() {
    // check if we are already at the right position
    if (Superstructure.atPosition(Superstructure.getSuperstructureState(), setpoint))
      state = SuperstructureCommandState.EXECUTING_ACTION;
  }

  @Override
  public void execute() {
    switch (state) {
      case TO_SAFE_ANGLE -> {
        if (Superstructure.isWristAtSafeAngle())
          state = SuperstructureCommandState.TO_ELEVATOR_SETPOINT;

        Subsystems.wrist.setWristSetpoint(setpoint.wristRotation());
        Subsystems.elevator.elevatorHold();
        Subsystems.roller.setVelocity(RotationsPerSecond.zero());
      }
      case TO_ELEVATOR_SETPOINT -> {
        if (MathUtil.isNear(
            Subsystems.elevator.getPercentage().baseUnitMagnitude(),
            setpoint.elevatorProportion().baseUnitMagnitude(),
            ElevatorConstants.elevatorTolerance.baseUnitMagnitude()))
          state = SuperstructureCommandState.TO_WRIST_SETPOINT;

        if (Superstructure.isWristAtSafeAngle()) {
          Subsystems.wrist.setWristSetpoint(setpoint.wristRotation());
        } else {
          Subsystems.wrist.setWristDutyCycle(Percent.zero());
        }
        Subsystems.elevator.setPercentageRaw(setpoint.elevatorProportion());
        Subsystems.roller.setVelocity(RotationsPerSecond.zero());
      }
      case TO_WRIST_SETPOINT -> {
        if (Superstructure.atPosition(Superstructure.getSuperstructureState(), setpoint))
          state = SuperstructureCommandState.EXECUTING_ACTION;

        Subsystems.wrist.setWristSetpoint(setpoint.wristRotation());
        Subsystems.elevator.elevatorHold();
        Subsystems.roller.setVelocity(RotationsPerSecond.zero());
      }
      case EXECUTING_ACTION -> {
        if (endCondition.getAsBoolean()) state = SuperstructureCommandState.DONE;

        Subsystems.wrist.setWristSetpoint(setpoint.wristRotation());
        Subsystems.elevator.elevatorHold();
        Subsystems.roller.setVelocity(setpoint.wheelSpeed());
      }
      case DONE -> {
        // at this point the command will be ended, but we do these just to make sure nothing
        // strange happens.
        Subsystems.wrist.setWristSetpoint(setpoint.wristRotation());
        Subsystems.elevator.elevatorHold();
        Subsystems.roller.setVelocity(setpoint.wheelSpeed());
      }
    }
  }

  @Override
  public void end(boolean interrupted) {
    super.end(interrupted);
  }

  @Override
  public boolean isFinished() {
    return state == SuperstructureCommandState.DONE;
  }
}
