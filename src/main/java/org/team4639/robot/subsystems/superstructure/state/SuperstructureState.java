package org.team4639.robot.subsystems.superstructure.state;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.units.measure.Dimensionless;
import lombok.Getter;

@Getter
public class SuperstructureState extends SuperstructureStateBase {
  private final Dimensionless elevatorProportion;
  private final Rotation2d wristRotation;
  private final AngularVelocity wheelSpeed;

  public SuperstructureState(
      Dimensionless elevatorProportion, Rotation2d wristRotation, AngularVelocity wheelSpeed) {
    this.elevatorProportion = elevatorProportion;
    this.wristRotation = wristRotation;
    this.wheelSpeed = wheelSpeed;
  }
}
