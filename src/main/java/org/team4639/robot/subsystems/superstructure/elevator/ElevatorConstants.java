package org.team4639.robot.subsystems.superstructure.elevator;

import static edu.wpi.first.units.Units.*;

import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.Dimensionless;
import edu.wpi.first.units.measure.Distance;
import org.team4639.lib.unit.UnitConverter;

public class ElevatorConstants {
  public static final Distance HEIGHT_MAX = Inches.of(0);
  public static final Distance HEIGHT_MIN = Inches.of(0);

  public static final Angle MOTOR_MAX = Rotations.of(0);
  public static final Angle MOTOR_MIN = Rotations.of(0);

  public static final Dimensionless PROPORTION_MAX = Percent.of(100);
  public static final Dimensionless PROPORTION_MIN = Percent.of(0);

  public static final UnitConverter<Distance, Angle> heightToRotations =
      UnitConverter.create(
              Distance::baseUnitMagnitude, d -> Distance.ofBaseUnits(d, Meter.one().baseUnit()))
          .then(
              UnitConverter.linearConvertingRange(
                  HEIGHT_MIN.baseUnitMagnitude(),
                  HEIGHT_MAX.baseUnitMagnitude(),
                  MOTOR_MIN.baseUnitMagnitude(),
                  MOTOR_MAX.baseUnitMagnitude()))
          .then(
              UnitConverter.create(
                  d -> Angle.ofBaseUnits(d, Rotations.getBaseUnit()), Angle::baseUnitMagnitude));

  public static final UnitConverter<Distance, Dimensionless> heightToPercentage =
      UnitConverter.create(
              Distance::baseUnitMagnitude, d -> Distance.ofBaseUnits(d, Meter.one().baseUnit()))
          .then(
              UnitConverter.linearConvertingRange(
                  HEIGHT_MIN.baseUnitMagnitude(),
                  HEIGHT_MAX.baseUnitMagnitude(),
                  PROPORTION_MIN.baseUnitMagnitude(),
                  PROPORTION_MAX.baseUnitMagnitude()))
          .then(
              UnitConverter.create(
                  d -> Dimensionless.ofBaseUnits(d, Value.getBaseUnit()),
                  Dimensionless::baseUnitMagnitude));
}
