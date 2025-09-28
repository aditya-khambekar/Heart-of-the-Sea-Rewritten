package org.team4639.robot.subsystems.superstructure.state;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.units.measure.Dimensionless;

public abstract class SuperstructureStateBase {
  public abstract Dimensionless getElevatorProportion();

  public abstract Rotation2d getWristRotation();

  public abstract AngularVelocity getWheelSpeed();
}
