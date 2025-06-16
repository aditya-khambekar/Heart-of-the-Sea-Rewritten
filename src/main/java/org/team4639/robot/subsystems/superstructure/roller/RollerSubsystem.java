package org.team4639.robot.subsystems.superstructure.roller;

import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.units.measure.Dimensionless;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import org.team4639.robot.subsystems.superstructure.roller.io.RollerIO;

public class RollerSubsystem extends SubsystemBase {
  RollerIO.RollerIOInputs inputs;
  RollerIO io;

  public RollerSubsystem(RollerIO io) {
    inputs = new RollerIO.RollerIOInputs();
    this.io = io;
  }

  @Override
  public void periodic() {
    io.updateInputs(inputs);
  }

  public void setVelocity(AngularVelocity velocity) {
    io.setVelocity(velocity);
  }

  public void setDutyCycle(Dimensionless percent) {
    io.setDutyCycleOutput(percent);
  }

  public AngularVelocity getVelocity() {
    return inputs.motorVelocity;
  }
}
