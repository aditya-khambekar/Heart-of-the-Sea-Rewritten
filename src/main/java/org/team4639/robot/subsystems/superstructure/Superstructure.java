package org.team4639.robot.subsystems.superstructure;

import static edu.wpi.first.units.Units.Rotations;
import static edu.wpi.first.units.Units.Value;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.units.measure.MutAngularVelocity;
import edu.wpi.first.util.sendable.Sendable;
import edu.wpi.first.util.sendable.SendableBuilder;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import org.team4639.robot.robot.Subsystems;
import org.team4639.robot.subsystems.superstructure.elevator.ElevatorConstants;
import org.team4639.robot.subsystems.superstructure.wrist.WristConstants;

/**
 * Does literally nothing except as a way to impose default commands on the elevator, wrist, and
 * roller and provide helper methods for controlling the superstructure.
 */
public class Superstructure extends SubsystemBase implements Sendable {
  /**
   * Mutates the given AngularVelocity to correspond to intaking a coral
   *
   * @param magnitude the magnitude of angular velocity. Should be positive
   */
  public static AngularVelocity coralIn(MutAngularVelocity magnitude) {
    return magnitude.mut_times(1);
  }

  /**
   * Mutates the given AngularVelocity to correspond to outtaking a coral
   *
   * @param magnitude the magnitude of angular velocity. Should be positive
   */
  public static AngularVelocity coralOut(MutAngularVelocity magnitude) {
    return magnitude.mut_times(-1);
  }

  /**
   * Mutates the given AngularVelocity to correspond to intaking an algae
   *
   * @param magnitude the magnitude of angular velocity. Should be positive
   */
  public static AngularVelocity algaeIn(MutAngularVelocity magnitude) {
    return magnitude.mut_times(-1);
  }

  /**
   * Mutates the given AngularVelocity to correspond to outtaking an algae
   *
   * @param magnitude the magnitude of angular velocity. Should be positive
   */
  public static AngularVelocity algaeOut(MutAngularVelocity magnitude) {
    return magnitude.mut_times(1);
  }

  /**
   * Checks if the superstructure is already at the positional state it needs to be in. Is used as a
   * preliminary check to skip straight to {@link
   * org.team4639.robot.commands.superstructure.SuperstructureCommandState#EXECUTING_ACTION} when
   * creating a new command.
   *
   * @param current current superstructure state
   * @param target target superstructure state
   */
  public static boolean atPosition(SuperstructureState current, SuperstructureState target) {
    return MathUtil.isNear(
            current.elevatorProportion().baseUnitMagnitude(),
            target.elevatorProportion().baseUnitMagnitude(),
            Math.abs(ElevatorConstants.elevatorTolerance.baseUnitMagnitude()))
        && MathUtil.isNear(
            current.wristRotation().getRotations(),
            target.wristRotation().getRotations(),
            Math.abs(WristConstants.wristTolerance.in(Rotations)));
  }

  public static SuperstructureState getSuperstructureState() {
    return new SuperstructureState(
        Subsystems.elevator.getPercentage(),
        Subsystems.wrist.getWristAngle(),
        Subsystems.roller.getVelocity());
  }

  /** Determines if the wrist is at a safe angle to move the elevator. */
  public static boolean isWristAtSafeAngle() {
    var max =
        WristConstants.SAFE_TRANSITION_RANGE.getFirst().getDegrees()
                > WristConstants.SAFE_TRANSITION_RANGE.getSecond().getDegrees()
            ? WristConstants.SAFE_TRANSITION_RANGE.getFirst()
            : WristConstants.SAFE_TRANSITION_RANGE.getSecond();
    var min =
        WristConstants.SAFE_TRANSITION_RANGE.getFirst().getDegrees()
                <= WristConstants.SAFE_TRANSITION_RANGE.getSecond().getDegrees()
            ? WristConstants.SAFE_TRANSITION_RANGE.getFirst()
            : WristConstants.SAFE_TRANSITION_RANGE.getSecond();
    ;

    var wristRotation = Subsystems.wrist.getWristAngle();

    var ret =
        wristRotation.getRadians() > min.getRadians()
            && wristRotation.getRadians() < max.getRadians();

    SmartDashboard.putBoolean("Wrist Safe", ret);
    return ret;
  }

  public static boolean isAtSafeAngle(Rotation2d rotation) {
    var max =
        WristConstants.SAFE_TRANSITION_RANGE.getFirst().getDegrees()
                > WristConstants.SAFE_TRANSITION_RANGE.getSecond().getDegrees()
            ? WristConstants.SAFE_TRANSITION_RANGE.getFirst()
            : WristConstants.SAFE_TRANSITION_RANGE.getSecond();
    var min =
        WristConstants.SAFE_TRANSITION_RANGE.getFirst().getDegrees()
                <= WristConstants.SAFE_TRANSITION_RANGE.getSecond().getDegrees()
            ? WristConstants.SAFE_TRANSITION_RANGE.getFirst()
            : WristConstants.SAFE_TRANSITION_RANGE.getSecond();
    ;

    return rotation.getRadians() > min.getRadians() && rotation.getRadians() < max.getRadians();
  }

  @Override
  public void initSendable(SendableBuilder builder) {
    builder.setSmartDashboardType("Superstructure");
    builder.addDoubleProperty(
        "Elevator Proportion", () -> getSuperstructureState().elevatorProportion().in(Value), null);
    builder.addDoubleProperty(
        "Wrist Angle Degrees", () -> getSuperstructureState().wristRotation().getDegrees(), null);
  }
}
