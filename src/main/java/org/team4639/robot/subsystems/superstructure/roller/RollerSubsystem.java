package org.team4639.robot.subsystems.superstructure.roller;

import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.units.measure.Dimensionless;
import edu.wpi.first.units.measure.Voltage;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import org.team4639.robot.subsystems.superstructure.roller.io.RollerIO;
import org.team4639.robot.subsystems.superstructure.roller.io.RollerIO.RollerIOInputs;

public class RollerSubsystem extends SubsystemBase {
  RollerIO.RollerIOInputs inputs;
  RollerIO io;

  public RollerSubsystem(RollerIO io) {
    inputs = new RollerIO.RollerIOInputs();
    this.io = io;

    SmartDashboard.putData("Roller Inputs", inputs);
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

  public void setVoltage(Voltage voltage) {
    io.setInputVoltage(voltage);
  }

  public AngularVelocity getVelocity() {
    return inputs.motorVelocity;
  }

  public RollerIOInputs getInputs() {
    return inputs;
  }
}
