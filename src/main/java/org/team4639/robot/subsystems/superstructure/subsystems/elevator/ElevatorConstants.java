package org.team4639.robot.subsystems.superstructure.subsystems.elevator;

import static edu.wpi.first.units.Units.*;

import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.Dimensionless;
import edu.wpi.first.units.measure.Distance;
import org.team4639.lib.tunable.*;
import org.team4639.lib.unit.UnitConverter;

public class ElevatorConstants {
  public static final Distance HEIGHT_MAX = Inches.of(90);
  public static final Distance HEIGHT_MIN = Inches.of(15);

  public static final Angle MOTOR_MAX = Rotations.of(60.91);
  public static final Angle MOTOR_MIN = Rotations.of(0);

  public static final Dimensionless PROPORTION_MAX = Percent.of(100);
  public static final Dimensionless PROPORTION_MIN = Percent.of(0);

  private static final UnitConverter<Double, Angle> DoubleToRotations =
      UnitConverter.create(
          d -> Angle.ofBaseUnits(d, Rotations.getBaseUnit()), Angle::baseUnitMagnitude);

  private static final UnitConverter<Distance, Double> heightToDouble =
      UnitConverter.create(
          Distance::baseUnitMagnitude, d -> Distance.ofBaseUnits(d, Meter.one().baseUnit()));

  private static final UnitConverter<Double, Dimensionless> DoubleToPercentage =
      UnitConverter.create(
          d -> Dimensionless.ofBaseUnits(d, Value.getBaseUnit()), Dimensionless::baseUnitMagnitude);

  public static final UnitConverter<Distance, Angle> heightToRotations =
      heightToDouble
          .then(
              UnitConverter.linearConvertingRange(
                  HEIGHT_MIN.baseUnitMagnitude(),
                  HEIGHT_MAX.baseUnitMagnitude(),
                  MOTOR_MIN.baseUnitMagnitude(),
                  MOTOR_MAX.baseUnitMagnitude()))
          .then(DoubleToRotations);

  public static final UnitConverter<Distance, Dimensionless> heightToPercentage =
      heightToDouble
          .then(
              UnitConverter.linearConvertingRange(
                  HEIGHT_MIN.baseUnitMagnitude(),
                  HEIGHT_MAX.baseUnitMagnitude(),
                  PROPORTION_MIN.baseUnitMagnitude(),
                  PROPORTION_MAX.baseUnitMagnitude()))
          .then(DoubleToPercentage);

  public static final UnitConverter<Angle, Dimensionless> rotationsToPercentage =
      DoubleToRotations.inverted()
          .then(
              UnitConverter.linearConvertingRange(
                  MOTOR_MIN.baseUnitMagnitude(),
                  MOTOR_MAX.baseUnitMagnitude(),
                  PROPORTION_MIN.baseUnitMagnitude(),
                  PROPORTION_MAX.baseUnitMagnitude()))
          .then(DoubleToPercentage);

  public static TunableNumber elevatorKp =
      new TunableNumber().withDefaultValue(3.596).send("Scoring PIDs/Elevator kp");
  public static TunableNumber elevatorKi =
      new TunableNumber().withDefaultValue(0.0).send("Scoring PIDs/Elevator kI");
  public static TunableNumber elevatorKd =
      new TunableNumber().withDefaultValue(0.0).send("Scoring PIDs/Elevator kD");
  public static TunableNumber elevatorVelocity =
      new TunableNumber().withDefaultValue(300.0).send("Scoring PIDs/Elevator Velocity");
  public static TunableNumber elevatorAcceleration =
      new TunableNumber().withDefaultValue(600).send("Scoring PIDs/Elevator Acceleration");
  public static TunableNumber elevatorKs =
      new TunableNumber().withDefaultValue(0.27919).send("Scoring PIDs/Elevator Ks");
  public static TunableNumber elevatorKg =
      new TunableNumber().withDefaultValue(0.2).send("Scoring PIDs/Elevator Kg");
  public static TunableNumber elevatorKv =
      new TunableNumber().withDefaultValue(0.11358).send("Scoring PIDs/Elevator Kv");
  public static TunableNumber elevatorKa =
      new TunableNumber().withDefaultValue(0.0).send("Scoring PIDs/Elevator Ka");

  public static final Dimensionless elevatorTolerance = Percent.of(0.5);

  /**
   * The point at which we can expand the wrist safe zone to include the full exterior range of
   * motion
   */
  public static final Dimensionless SAFE_ZONE_EXPANSION = Percent.of(25);
}
