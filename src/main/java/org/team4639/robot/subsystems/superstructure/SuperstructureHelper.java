package org.team4639.robot.subsystems.superstructure;

import static edu.wpi.first.units.Units.Rotations;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.units.measure.MutAngularVelocity;
import java.util.Arrays;
import java.util.Comparator;
import org.team4639.robot.robot.Subsystems;
import org.team4639.robot.subsystems.superstructure.elevator.ElevatorConstants;
import org.team4639.robot.subsystems.superstructure.wrist.WristConstants;

public class SuperstructureHelper {
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
            ElevatorConstants.elevatorTolerance.baseUnitMagnitude())
        && MathUtil.isNear(
            current.wristRotation().getRotations(),
            target.wristRotation().getRotations(),
            WristConstants.wristTolerance.in(Rotations));
  }

  public static SuperstructureState getSuperstructureState() {
    return new SuperstructureState(
        Subsystems.elevator.getPercentage(),
        Subsystems.wrist.getWristAngle(),
        Subsystems.roller.getVelocity());
  }

  /** Determines if the wrist is at a safe angle to move the elevator. */
  public static boolean isWristAtSafeAngle() {
    Rotation2d[] ordered =
        (Rotation2d[])
            Arrays.stream(WristConstants.SAFE_TRANSITION_RANGE)
                .sorted(Comparator.comparingDouble(Rotation2d::getRadians))
                .toArray();
    var max = ordered[1];
    var min = ordered[0];

    var wristRotation = Subsystems.wrist.getWristAngle();

    return wristRotation.getRadians() > min.getRadians()
        && wristRotation.getRadians() < max.getRadians();
  }
}
