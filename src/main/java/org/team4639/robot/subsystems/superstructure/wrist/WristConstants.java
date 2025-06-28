package org.team4639.robot.subsystems.superstructure.wrist;

import static edu.wpi.first.units.Units.*;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.Current;
import org.team4639.lib.annotation.Untuned;
import org.team4639.lib.unit.UnitConverter;

public class WristConstants {
  public static final Rotation2d MAX_ROTATION = Rotation2d.fromDegrees(-200);
  public static final Rotation2d IDLE_ROTATION = Rotation2d.fromDegrees(30);

  @Untuned public static final Angle MAX_POSITION = Rotations.of(-31.846);
  @Untuned public static final Angle MIN_POSITION = Rotations.of(0);

  public static final Rotation2d TRANSITION_ROTATION = Rotation2d.fromDegrees(-85);

  public static final Current ALGAE_CURRENT = Amps.of(1);

  @Untuned
  public static final Rotation2d[] SAFE_TRANSITION_RANGE =
      new Rotation2d[] {
        TRANSITION_ROTATION.plus(Rotation2d.fromDegrees(20)),
        TRANSITION_ROTATION.minus(Rotation2d.fromDegrees(20)),
      };

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
