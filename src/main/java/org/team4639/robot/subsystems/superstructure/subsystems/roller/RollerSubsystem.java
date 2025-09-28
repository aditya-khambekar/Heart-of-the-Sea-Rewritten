package org.team4639.robot.subsystems.superstructure.subsystems.roller;

import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.units.measure.Dimensionless;
import edu.wpi.first.units.measure.Voltage;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import lombok.Getter;
import org.team4639.robot.subsystems.superstructure.subsystems.roller.io.RollerIO;

@Getter
public class RollerSubsystem extends SubsystemBase {
  @Getter private final RollerIO.RollerIOInputs inputs;
  private final RollerIO io;

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
}
