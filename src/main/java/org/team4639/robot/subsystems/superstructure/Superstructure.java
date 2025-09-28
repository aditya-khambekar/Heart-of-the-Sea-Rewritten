package org.team4639.robot.subsystems.superstructure;

import static edu.wpi.first.units.Units.Inches;
import static edu.wpi.first.units.Units.Meters;
import static edu.wpi.first.units.Units.Rotations;
import static edu.wpi.first.units.Units.Value;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.Pair;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.units.measure.Distance;
import edu.wpi.first.units.measure.MutAngularVelocity;
import edu.wpi.first.util.sendable.Sendable;
import edu.wpi.first.util.sendable.SendableBuilder;
import edu.wpi.first.wpilibj.smartdashboard.Mechanism2d;
import edu.wpi.first.wpilibj.smartdashboard.MechanismLigament2d;
import edu.wpi.first.wpilibj.smartdashboard.MechanismRoot2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj.util.Color8Bit;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import java.util.Optional;
import org.team4639.lib.unit.Units2;
import org.team4639.lib.util.RotationUtil;
import org.team4639.robot.robot.Subsystems;
import org.team4639.robot.subsystems.superstructure.state.SuperstructureState;
import org.team4639.robot.subsystems.superstructure.subsystems.elevator.ElevatorConstants;
import org.team4639.robot.subsystems.superstructure.subsystems.wrist.WristConstants;

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
            current.getElevatorProportion().baseUnitMagnitude(),
            target.getElevatorProportion().baseUnitMagnitude(),
            Math.abs(ElevatorConstants.elevatorTolerance.baseUnitMagnitude()))
        && MathUtil.isNear(
            current.getWristRotation().getRotations(),
            target.getWristRotation().getRotations(),
            Math.abs(WristConstants.wristTolerance.in(Rotations)));
  }

  public static boolean wristAtSetpoint(SuperstructureState current, SuperstructureState target) {
    return MathUtil.isNear(
        current.getWristRotation().getRotations(),
        target.getWristRotation().getRotations(),
        Math.abs(WristConstants.wristTolerance.in(Rotations)));
  }

  public static boolean atPosition(SuperstructureState target) {
    var current = getSuperstructureState();
    return MathUtil.isNear(
            current.getElevatorProportion().baseUnitMagnitude(),
            target.getElevatorProportion().baseUnitMagnitude(),
            Math.abs(ElevatorConstants.elevatorTolerance.baseUnitMagnitude()))
        && MathUtil.isNear(
            current.getWristRotation().getRotations(),
            target.getWristRotation().getRotations(),
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
    var zone = getEffectiveExteriorSafeZone();
    var ret =
        RotationUtil.boundedBy(Subsystems.wrist.getWristAngle(), zone.getFirst(), zone.getSecond());
    SmartDashboard.putBoolean("Wrist Safe", ret);
    return ret;
  }

  /** Determines if the wrist is at a safe angle to move the elevator. */
  public static boolean isWristAtSafeAngle(Rotation2d wristRotation) {
    var zone = getEffectiveExteriorSafeZone();
    var ret = RotationUtil.boundedBy(wristRotation, zone.getFirst(), zone.getSecond());
    SmartDashboard.putBoolean("Wrist Safe", ret);
    return ret;
  }

  public static Pair<Rotation2d, Rotation2d> getEffectiveExteriorSafeZone() {
    return Subsystems.elevator.getPercentage().compareTo(ElevatorConstants.SAFE_ZONE_EXPANSION) >= 0
        ? WristConstants.SAFE_TRANSITION_RANGE_HIGH
        : WristConstants.SAFE_TRANSITION_RANGE_LOW;
  }

  @Override
  public void initSendable(SendableBuilder builder) {
    builder.setSmartDashboardType("Superstructure");
    builder.addDoubleProperty(
        "Elevator Proportion",
        () -> getSuperstructureState().getElevatorProportion().in(Value),
        null);
    builder.addDoubleProperty(
        "Wrist Angle Degrees",
        () -> getSuperstructureState().getWristRotation().getDegrees(),
        null);
  }

  public static final double height = 120;
  public static final double width = 28.5;

  public static final java.awt.Color currentColor = new java.awt.Color(16, 125, 215);
  public static final java.awt.Color targetColor = new java.awt.Color(0, 178, 99);

  public static final Distance hopperLength = Inches.of(14);

  public static final Translation2d origin =
      new Translation2d(Units.inchesToMeters(width / 2 + 6.25), 0.0);

  public static final Mechanism2d mechanismView =
      new Mechanism2d(Units.inchesToMeters(width), Units.inchesToMeters(height));

  public static final MechanismRoot2d elevatorRoot =
      mechanismView.getRoot("Current Elevator Root", origin.getX(), origin.getY());

  public static final MechanismLigament2d currentElevatorLigament =
      elevatorRoot.append(
          new MechanismLigament2d(
              "Current Elevator Ligament",
              ElevatorConstants.heightToPercentage
                      .convertBackwards(Subsystems.elevator.getPercentage())
                      .in(Meters)
                  + Units2.inchesToMeters.convert(4),
              90,
              4,
              new Color8Bit(
                  currentColor.getRed(), currentColor.getGreen(), currentColor.getBlue())));

  public static final MechanismLigament2d currentHopperLigament =
      currentElevatorLigament.append(
          new MechanismLigament2d(
              "Current Hopper Ligament",
              hopperLength.in(Meters),
              0,
              4,
              new Color8Bit(
                  currentColor.getRed(), currentColor.getGreen(), currentColor.getBlue())));

  @Override
  public void periodic() {
    currentElevatorLigament.setLength(
        ElevatorConstants.heightToPercentage
            .convertBackwards(Subsystems.elevator.getPercentage())
            .in(Meters));
    currentHopperLigament.setAngle(
        Optional.of(Subsystems.wrist.getWristAngle().minus(Rotation2d.kCCW_90deg))
            .map(x -> Rotation2d.kZero.minus(x.minus(Rotation2d.kZero)))
            .get());
    SmartDashboard.putData("Superstructure", mechanismView);
  }
}
