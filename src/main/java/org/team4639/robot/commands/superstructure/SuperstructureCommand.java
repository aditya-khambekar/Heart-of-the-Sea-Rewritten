package org.team4639.robot.commands.superstructure;

import static edu.wpi.first.units.Units.Percent;
import static edu.wpi.first.units.Units.Rotations;
import static edu.wpi.first.units.Units.RotationsPerSecond;
import static edu.wpi.first.units.Units.Seconds;
import static edu.wpi.first.units.Units.Value;
import static edu.wpi.first.units.Units.Volts;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.Pair;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.units.measure.Dimensionless;
import edu.wpi.first.units.measure.MutTime;
import edu.wpi.first.units.measure.Time;
import edu.wpi.first.units.measure.Voltage;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import java.util.function.BooleanSupplier;
import org.team4639.lib.util.RotationUtil;
import org.team4639.robot.constants.Controls;
import org.team4639.robot.robot.RobotContainer;
import org.team4639.robot.robot.Subsystems;
import org.team4639.robot.subsystems.superstructure.Superstructure;
import org.team4639.robot.subsystems.superstructure.SuperstructureState;
import org.team4639.robot.subsystems.superstructure.elevator.ElevatorConstants;
import org.team4639.robot.subsystems.superstructure.wrist.WristConstants;

public class SuperstructureCommand extends SuperstructureCommandBase {
  private SuperstructureState setpoint;
  private SuperstructureCommandState state;
  private BooleanSupplier endCondition;
  private Dimensionless holdPosition;
  private MutTime timeOfExecutingAction;
  private Time executingActionTimeout = Seconds.of(Double.POSITIVE_INFINITY);
  private String name;
  private boolean flash;
  private boolean wristSupposedToBeStopped = true;
  private Voltage whileRunningRollerVolts = Volts.of(0.0);
  private boolean waitToRoller = false;
  private Trigger waitForRollerTrigger = Controls.ROLLER_TRIGGER;
  private boolean internalForceRoller = false;
  boolean coral = false;

  /**
   * Commands the superstructure to go to a specific state
   *
   * @param setpoint the setpoint that the superstructure is commanded to
   * @param endCondition the condition to end this command. Ex. has coral, doesn't have coral, or
   *     can always return false for a command that doesn't end by itself. This is checked only once
   *     the command has reached the EXECUTING_ACTION state.
   */
  public SuperstructureCommand(
      SuperstructureState setpoint, BooleanSupplier endCondition, String name) {
    addRequirements(
        Subsystems.elevator, Subsystems.wrist, Subsystems.roller, Subsystems.superstructure);
    this.setpoint = setpoint;
    this.state = SuperstructureCommandState.TO_SAFE_ANGLE;
    this.endCondition = endCondition;
    holdPosition = Value.zero();
    timeOfExecutingAction = Seconds.mutable(0);
    this.name = name;
    setName(name);
  }

  public SuperstructureCommand(SuperstructureState setpoint, String name) {
    this(setpoint, () -> false, name);
  }

  @Override
  public void initialize() {
    internalForceRoller = false;
    setState(SuperstructureCommandState.TO_SAFE_ANGLE);
    if (Superstructure.atPosition(Superstructure.getSuperstructureState(), setpoint))
      setState(SuperstructureCommandState.EXECUTING_ACTION);
    timeOfExecutingAction = Seconds.mutable(0);
  }

  @Override
  public void execute() {
    SmartDashboard.putNumber("Wrist Setpoint", setpoint.wristRotation().getDegrees());
    super.execute();
    SmartDashboard.putBoolean("Elevator At Setpoint", elevatorAtSetpoint());
    SmartDashboard.putNumber("Elevator Setpoint", setpoint.elevatorProportion().in(Value));
    SmartDashboard.putNumber("Elevator Position", Subsystems.elevator.getPercentage().in(Value));
    if (Superstructure.atPosition(Superstructure.getSuperstructureState(), setpoint))
      setState(SuperstructureCommandState.EXECUTING_ACTION);
    switch (state) {
      case TO_SAFE_ANGLE -> {
        if (elevatorAtSetpoint()) setState(SuperstructureCommandState.TO_WRIST_SETPOINT);
        if (Superstructure.isWristAtSafeAngle() || willWristBeSafe())
          setState(SuperstructureCommandState.TO_ELEVATOR_SETPOINT);
        if (RotationUtil.boundedBy(
                Subsystems.wrist.getWristAngle(),
                WristConstants.SAFE_TRANSITION_RANGE_INTERIOR.getFirst(),
                WristConstants.SAFE_TRANSITION_RANGE_INTERIOR.getSecond())
            && RotationUtil.boundedBy(
                setpoint.wristRotation(),
                WristConstants.SAFE_TRANSITION_RANGE_INTERIOR.getFirst(),
                WristConstants.SAFE_TRANSITION_RANGE_INTERIOR.getSecond()))
          setState(SuperstructureCommandState.TO_ELEVATOR_SETPOINT);
        wristSupposedToBeStopped = false;
        Pair<Rotation2d, Rotation2d> safeTransitionRange =
            Superstructure.getEffectiveExteriorSafeZone();
        Subsystems.wrist.setWristSetpoint(
            RotationUtil.nearest(
                setpoint.wristRotation(),
                RotationUtil.min(safeTransitionRange.getFirst(), safeTransitionRange.getSecond())
                    .plus(Rotation2d.fromDegrees(2)),
                RotationUtil.max(safeTransitionRange.getFirst(), safeTransitionRange.getSecond())
                    .minus(Rotation2d.fromDegrees(2))));
        Subsystems.elevator.setPercentageRaw(holdPosition);
        runRollerVelo(whileRunningRollerVolts);
      }
      case TO_ELEVATOR_SETPOINT -> {
        if (elevatorAtSetpoint()) setState(SuperstructureCommandState.TO_WRIST_SETPOINT);
        if (Superstructure.isWristAtSafeAngle()) {
          Subsystems.wrist.setWristSetpoint(setpoint.wristRotation());
          wristSupposedToBeStopped = false;
        } else {
          Subsystems.wrist.setWristDutyCycle(Percent.zero());
          wristSupposedToBeStopped = true;
        }
        Subsystems.elevator.setPercentageRaw(setpoint.elevatorProportion());
        runRollerVelo(whileRunningRollerVolts);
      }
      case TO_WRIST_SETPOINT -> {
        wristSupposedToBeStopped = false;
        if (Superstructure.atPosition(Superstructure.getSuperstructureState(), setpoint))
          setState(SuperstructureCommandState.EXECUTING_ACTION);

        Subsystems.wrist.setWristSetpoint(setpoint.wristRotation());
        Subsystems.elevator.setPercentageRaw(setpoint.elevatorProportion());
        ;
        runRollerVelo(whileRunningRollerVolts);
      }
      case EXECUTING_ACTION -> {
        wristSupposedToBeStopped = true;
        timeOfExecutingAction.mut_plus(0.02, Seconds);
        if (endCondition.getAsBoolean()) setState(SuperstructureCommandState.DONE);
        if (timeOfExecutingAction.gte(executingActionTimeout))
          setState(SuperstructureCommandState.DONE);
        if (flash) Subsystems.limelightFlash.flash();

        if (waitToRoller) {
          if (waitForRollerTrigger.getAsBoolean()) {
            runRollerVelo();
          }
        } else {
          if (coral) {
            if (timeOfExecutingAction.gte(Seconds.of(0.2))) {
              runRollerVelo();
            }
          } else {
            runRollerVelo();
          }
        }
        if (coral) {
          if (timeOfExecutingAction.gte(Seconds.of(0.2))) {
            runRollerVelo();
          }
        } else {
          runRollerVelo();
        }
        Subsystems.wrist.setWristSetpoint(setpoint.wristRotation());
        Subsystems.elevator.setPercentageRaw(setpoint.elevatorProportion());
      }
      case DONE -> {
        // at this point the command will be ended, but we do these just to make sure nothing
        // strange happens.
        Subsystems.wrist.setWristSetpoint(setpoint.wristRotation());
        Subsystems.elevator.setPercentageRaw(setpoint.elevatorProportion());
        ;
        Subsystems.roller.setVelocity(setpoint.wheelSpeed());
      }
      case STOPPED -> {
        Subsystems.wrist.setWristDutyCycle(Value.zero());
        Subsystems.elevator.elevatorStop();
        Subsystems.roller.setDutyCycle(Value.zero());
      }
      default -> throw new IllegalArgumentException("Unexpected state: " + state);
    }
  }

  public SuperstructureCommand flashOnDone() {
    this.flash = true;
    return this;
  }

  public SuperstructureCommand withCoral() {
    this.whileRunningRollerVolts = Volts.of(-0.1);
    coral = true;
    return this;
  }

  public SuperstructureCommand withAlgae() {
    this.whileRunningRollerVolts = Volts.of(-5.0);
    return this;
  }

  public SuperstructureCommand withNone() {
    this.whileRunningRollerVolts = Volts.of(0.0);
    return this;
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

  public SuperstructureCommand withExecutionTimeout(Time time) {
    this.executingActionTimeout = time;
    return this;
  }

  @Override
  public SuperstructureCommandState getState() {
    return state;
  }

  @Override
  public String getName() {
    return name;
  }

  public boolean willWristBeSafe() {
    var lookahead =
        WristConstants.RotationToPosition.convertBackwards(
                Rotations.of(Subsystems.wrist.getEncoderVelocity().in(RotationsPerSecond)))
            .times(0.1);
    return Superstructure.isWristAtSafeAngle(Subsystems.wrist.getWristAngle().plus(lookahead));
  }

  public void runRollerVelo() {
    if (internalForceRoller || RobotContainer.driver.a().getAsBoolean()) {
      if (internalForceRoller) {
        Subsystems.roller.setVelocity(
            RotationsPerSecond.of(5).times(Math.signum(setpoint.wheelSpeed().magnitude())));
      } else {
        Subsystems.roller.setVelocity(
            RotationsPerSecond.of(5).times(Math.signum(setpoint.wheelSpeed().magnitude())));
      }

    } else {
      Subsystems.roller.setVelocity(setpoint.wheelSpeed());
    }
  }

  public void runRollerVelo(AngularVelocity velocity) {
    if (internalForceRoller || RobotContainer.driver.a().getAsBoolean()) {
      if (internalForceRoller) {
        Subsystems.roller.setVelocity(
            RotationsPerSecond.of(5).times(Math.signum(setpoint.wheelSpeed().magnitude())));
      } else {
        Subsystems.roller.setVelocity(
            RotationsPerSecond.of(5).times(Math.signum(setpoint.wheelSpeed().magnitude())));
      }
    } else {
      Subsystems.roller.setVelocity(velocity);
    }
  }

  public void runRollerVelo(Voltage volts) {
    if (internalForceRoller || RobotContainer.driver.a().getAsBoolean()) {
      if (internalForceRoller) {
        Subsystems.roller.setVelocity(
            RotationsPerSecond.of(5).times(Math.signum(setpoint.wheelSpeed().magnitude())));
      } else {
        Subsystems.roller.setVelocity(
            RotationsPerSecond.of(5).times(Math.signum(setpoint.wheelSpeed().magnitude())));
      }
    } else {
      Subsystems.roller.setVoltage(volts);
    }
  }

  public SuperstructureCommand waitForRoller() {
    this.waitToRoller = true;
    return this;
  }

  public SuperstructureCommand waitForRoller(Trigger trigger) {
    this.waitForRollerTrigger = trigger;
    return waitForRoller();
  }

  public void forceRoller() {
    internalForceRoller = true;
  }
}
