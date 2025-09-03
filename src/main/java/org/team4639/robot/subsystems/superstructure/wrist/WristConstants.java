package org.team4639.robot.subsystems.superstructure.wrist;

import static edu.wpi.first.units.Units.*;

import edu.wpi.first.math.Pair;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.Current;
import org.team4639.lib.annotation.Untuned;
import org.team4639.lib.unit.UnitConverter;

public class WristConstants {
  public static final Rotation2d MAX_ROTATION = Rotation2d.fromDegrees(230);
  public static final Rotation2d IDLE_ROTATION = Rotation2d.fromDegrees(30);

  @Untuned public static final Angle MAX_POSITION = Rotations.of(0.075);
  @Untuned public static final Angle MIN_POSITION = Rotations.of(0.988);

  public static final Rotation2d TRANSITION_ROTATION = Rotation2d.fromDegrees(145);

  public static final Current ALGAE_CURRENT = Amps.of(1);

  @Untuned
  public static final Pair<Rotation2d, Rotation2d> SAFE_TRANSITION_RANGE_LOW =
      new Pair<>(Rotation2d.fromDegrees(145), Rotation2d.fromDegrees(95));

  @Untuned
  public static final Pair<Rotation2d, Rotation2d> SAFE_TRANSITION_RANGE_HIGH =
      new Pair<>(MAX_ROTATION, Rotation2d.fromDegrees(95));

  @Untuned
  public static final Pair<Rotation2d, Rotation2d> SAFE_TRANSITION_RANGE_INTERIOR =
      new Pair<>(IDLE_ROTATION, Rotation2d.fromDegrees(80));

  public static final UnitConverter<Rotation2d, Angle> RotationToPosition =
      UnitConverter.create(Rotation2d::getDegrees, Rotation2d::fromDegrees)
          .then(
              UnitConverter.linearConvertingRange(
                  IDLE_ROTATION.getDegrees(),
                  MAX_ROTATION.getDegrees(),
                  MIN_POSITION.baseUnitMagnitude(),
                  MAX_POSITION.baseUnitMagnitude()))
          .then(
              UnitConverter.create(
                  d -> Angle.ofBaseUnits(d, Degree.getBaseUnit()), Angle::baseUnitMagnitude));

  public static final Angle wristTolerance = Degrees.of(1);
}
