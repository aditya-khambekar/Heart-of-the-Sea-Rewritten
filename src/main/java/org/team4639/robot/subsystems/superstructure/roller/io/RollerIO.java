package org.team4639.robot.subsystems.superstructure.roller.io;

import static edu.wpi.first.units.Units.*;

import edu.wpi.first.units.Units;
import edu.wpi.first.units.measure.*;
import edu.wpi.first.util.sendable.Sendable;
import edu.wpi.first.util.sendable.SendableBuilder;
import java.util.Optional;

public abstract class RollerIO {
  public static class RollerIOInputs implements Sendable {
    public MutAngularVelocity motorVelocity = RadiansPerSecond.mutable(0);
    public MutTemperature motorTemperature = Celsius.mutable(0);
    public MutCurrent motorCurrent = Amps.mutable(0);

    @Override
    public void initSendable(SendableBuilder builder) {
      builder.addDoubleProperty(
          "Motor Speed Rad/s",
          () ->
              Optional.ofNullable((AngularVelocity) motorVelocity)
                  .orElse(RadiansPerSecond.zero())
                  .in(RadiansPerSecond),
          null);
      builder.addDoubleProperty(
          "Motor Celsius",
          () ->
              Optional.ofNullable((Temperature) motorTemperature)
                  .orElse(Celsius.zero())
                  .in(Celsius),
          null);
      builder.addDoubleProperty(
          "Motor Current",
          () -> Optional.ofNullable((Current) motorCurrent).orElse(Amps.zero()).in(Units.Amp),
          null);
    }
  }

  public abstract void updateInputs(RollerIOInputs inputs);

  public abstract void setDutyCycleOutput(Dimensionless percent);

  public abstract void setVelocity(AngularVelocity velocity);
}
