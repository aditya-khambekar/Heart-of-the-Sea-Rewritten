package org.team4639.robot.subsystems.superstructure.roller.io;

import static edu.wpi.first.units.Units.*;

import edu.wpi.first.units.Units;
import edu.wpi.first.units.measure.*;
import edu.wpi.first.util.sendable.Sendable;
import edu.wpi.first.util.sendable.SendableBuilder;
import java.util.Optional;

public abstract class RollerIO {
  public static class RollerIOInputs implements Sendable {
    public AngularVelocity motorVelocity;
    public Temperature motorTemperature;
    public Current motorCurrent;

    @Override
    public void initSendable(SendableBuilder builder) {
      builder.addDoubleProperty(
          "Motor Speed Rad/s",
          () ->
              Optional.ofNullable(motorVelocity)
                  .orElse(RadiansPerSecond.zero())
                  .in(RadiansPerSecond),
          null);
      builder.addDoubleProperty(
          "Motor Celsius",
          () -> Optional.ofNullable(motorTemperature).orElse(Celsius.zero()).in(Celsius),
          null);
      builder.addDoubleProperty(
          "Motor Current",
          () -> Optional.ofNullable(motorCurrent).orElse(Amps.zero()).in(Units.Amp),
          null);
    }
  }

  public abstract void updateInputs(RollerIOInputs inputs);

  public abstract void setDutyCycleOutput(Dimensionless percent);

  public abstract void setVelocity(AngularVelocity velocity);
}
