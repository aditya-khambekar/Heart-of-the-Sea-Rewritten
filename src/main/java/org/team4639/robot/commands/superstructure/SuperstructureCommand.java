package org.team4639.robot.commands.superstructure;

import static edu.wpi.first.units.Units.Percent;
import static edu.wpi.first.units.Units.Rotations;
import static edu.wpi.first.units.Units.RotationsPerSecond;
import static edu.wpi.first.units.Units.Value;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.units.measure.Dimensionless;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import java.util.function.BooleanSupplier;
import org.team4639.lib.util.RotationUtil;
import org.team4639.robot.robot.Subsystems;
import org.team4639.robot.subsystems.superstructure.Superstructure;
import org.team4639.robot.subsystems.superstructure.SuperstructureState;
import org.team4639.robot.subsystems.superstructure.elevator.ElevatorConstants;
import org.team4639.robot.subsystems.superstructure.wrist.WristConstants;

public class SuperstructureCommand extends Command {
  private SuperstructureState setpoint;
  private SuperstructureCommandState state;
  private BooleanSupplier endCondition;
  private Dimensionless holdPosition;

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
    holdPosition = Value.zero();
  }

  public SuperstructureCommand(SuperstructureState setpoint) {
    this(setpoint, () -> false);
  }

  @Override
  public void initialize() {
    setState(SuperstructureCommandState.TO_SAFE_ANGLE);
  }

  @Override
  public void execute() {
    SmartDashboard.putString("Superstructure State", state.toString());
    SmartDashboard.putBoolean("Elevator At Setpoint", elevatorAtSetpoint());
    SmartDashboard.putNumber(
        "Command Elevator Setpoint",
        ElevatorConstants.rotationsToPercentage
            .convertBackwards(setpoint.elevatorProportion())
            .in(Rotations));
    SmartDashboard.putNumber(
        "Hold Rotations",
        ElevatorConstants.rotationsToPercentage.convertBackwards(holdPosition).in(Rotations));
    if (Superstructure.atPosition(Superstructure.getSuperstructureState(), setpoint))
      setState(SuperstructureCommandState.EXECUTING_ACTION);
    switch (state) {
      case TO_SAFE_ANGLE -> {
        if (elevatorAtSetpoint()) setState(SuperstructureCommandState.TO_WRIST_SETPOINT);
        if (Superstructure.isWristAtSafeAngle())
          setState(SuperstructureCommandState.TO_ELEVATOR_SETPOINT);

        Subsystems.wrist.setWristSetpoint(
            RotationUtil.nearest(
                setpoint.wristRotation(),
                RotationUtil.min(
                        WristConstants.SAFE_TRANSITION_RANGE.getFirst(),
                        WristConstants.SAFE_TRANSITION_RANGE.getSecond())
                    .plus(Rotation2d.fromDegrees(2)),
                RotationUtil.max(
                        WristConstants.SAFE_TRANSITION_RANGE.getFirst(),
                        WristConstants.SAFE_TRANSITION_RANGE.getSecond())
                    .minus(Rotation2d.fromDegrees(2))));
        Subsystems.elevator.setPercentageRaw(holdPosition);
        Subsystems.roller.setVelocity(RotationsPerSecond.zero());
      }
      case TO_ELEVATOR_SETPOINT -> {
        if (elevatorAtSetpoint()) setState(SuperstructureCommandState.TO_WRIST_SETPOINT);

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
          setState(SuperstructureCommandState.EXECUTING_ACTION);

        Subsystems.wrist.setWristSetpoint(setpoint.wristRotation());
        Subsystems.elevator.setPercentageRaw(setpoint.elevatorProportion());
        ;
        Subsystems.roller.setVelocity(RotationsPerSecond.zero());
      }
      case EXECUTING_ACTION -> {
        if (endCondition.getAsBoolean()) setState(SuperstructureCommandState.DONE);

        Subsystems.wrist.setWristSetpoint(setpoint.wristRotation());
        Subsystems.elevator.setPercentageRaw(setpoint.elevatorProportion());
        Subsystems.roller.setVelocity(setpoint.wheelSpeed());
      }
      case DONE -> {
        // at this point the command will be ended, but we do these just to make sure nothing
        // strange happens.
        Subsystems.wrist.setWristSetpoint(setpoint.wristRotation());
        Subsystems.elevator.setPercentageRaw(setpoint.elevatorProportion());
        ;
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

  private boolean elevatorAtSetpoint() {
    return MathUtil.isNear(
        Subsystems.elevator.getPercentage().baseUnitMagnitude(),
        setpoint.elevatorProportion().baseUnitMagnitude(),
        Math.abs(ElevatorConstants.elevatorTolerance.baseUnitMagnitude()));
  }

  private void setHoldPosition() {
    holdPosition = Subsystems.elevator.getPercentage();
  }

  private void setState(SuperstructureCommandState state) {
    this.state = state;
    setHoldPosition();
  }
}
