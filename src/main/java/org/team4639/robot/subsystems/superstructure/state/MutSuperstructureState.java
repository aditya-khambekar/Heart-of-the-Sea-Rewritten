package org.team4639.robot.subsystems.superstructure.state;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.units.measure.Dimensionless;
import lombok.Getter;
import lombok.Setter;

@Getter
@Setter
public class MutSuperstructureState extends SuperstructureStateBase {
  private Dimensionless elevatorProportion;
  private Rotation2d wristRotation;
  private AngularVelocity wheelSpeed;

  public MutSuperstructureState(
      Dimensionless elevatorProportion, Rotation2d wristRotation, AngularVelocity wheelSpeed) {
    this.elevatorProportion = elevatorProportion;
    this.wristRotation = wristRotation;
    this.wheelSpeed = wheelSpeed;
  }
}
