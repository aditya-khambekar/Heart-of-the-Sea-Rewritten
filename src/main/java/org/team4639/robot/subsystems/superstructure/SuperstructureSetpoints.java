package org.team4639.robot.subsystems.superstructure;

import static edu.wpi.first.units.Units.Percent;
import static edu.wpi.first.units.Units.RotationsPerSecond;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.units.measure.AngularVelocity;

public class SuperstructureSetpoints {
  private static final AngularVelocity coralIn =
      Superstructure.coralIn(RotationsPerSecond.mutable(5));
  private static final AngularVelocity coralOut =
      Superstructure.coralOut(RotationsPerSecond.mutable(8));

  private static final AngularVelocity algaeIn =
      Superstructure.algaeIn(RotationsPerSecond.mutable(8));
  private static final AngularVelocity algaeOut =
      Superstructure.algaeOut(RotationsPerSecond.mutable(8));

  public static final SuperstructureState intake =
      new SuperstructureState(Percent.zero(), Rotation2d.fromDegrees(-10.25), coralIn);

  public static final SuperstructureState homing_ready =
      new SuperstructureState(Percent.zero(), Rotation2d.fromDegrees(-10.25), coralIn);
}
