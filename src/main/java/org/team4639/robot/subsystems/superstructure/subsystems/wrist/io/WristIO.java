package org.team4639.robot.subsystems.superstructure.subsystems.wrist.io;

import static edu.wpi.first.units.Units.*;

import edu.wpi.first.units.Units;
import edu.wpi.first.units.measure.*;
import edu.wpi.first.util.sendable.Sendable;
import edu.wpi.first.util.sendable.SendableBuilder;
import java.util.Optional;
import lombok.Getter;

public abstract class WristIO {
  public static class WristIOInputs implements Sendable {
    @Getter public MutAngle motorPosition = Degrees.mutable(0);
    @Getter public MutAngularVelocity motorVelocity = RadiansPerSecond.mutable(0);
    @Getter public MutTemperature motorTemperature = Celsius.mutable(0);
    @Getter public MutCurrent motorCurrent = Amps.mutable(0);
    @Getter public MutVoltage motorVoltage = Volts.mutable(0);

    @Override
    public void initSendable(SendableBuilder builder) {
      builder.addDoubleProperty(
          "Motor Rotations",
          () -> Optional.ofNullable((Angle) motorPosition).orElse(Rotations.zero()).in(Rotations),
          null);
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

  public abstract void updateInputs(WristIOInputs inputs);

  public abstract void setDutyCycleOutput(Dimensionless percent);

  public abstract void setPosition(Angle position);

  public abstract void setVoltage(Voltage voltage);
}
