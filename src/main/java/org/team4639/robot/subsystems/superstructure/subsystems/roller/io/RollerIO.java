package org.team4639.robot.subsystems.superstructure.subsystems.roller.io;

import static edu.wpi.first.units.Units.*;

import edu.wpi.first.units.Units;
import edu.wpi.first.units.measure.*;
import edu.wpi.first.util.sendable.Sendable;
import edu.wpi.first.util.sendable.SendableBuilder;
import java.util.Optional;
import lombok.Getter;

public abstract class RollerIO {
  public static class RollerIOInputs implements Sendable {
    @Getter public MutAngularVelocity motorVelocity = RadiansPerSecond.mutable(0);
    @Getter public MutAngle motorPosition = Rotations.mutable(0);
    @Getter public MutVoltage voltage = Volts.mutable(0);
    @Getter public MutTemperature motorTemperature = Celsius.mutable(0);
    @Getter public MutCurrent motorCurrent = Amps.mutable(0);

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
          "Motor Position Rotations",
          () -> Optional.ofNullable((Angle) motorPosition).orElse(Rotations.zero()).in(Rotations),
          null);

      builder.addDoubleProperty(
          "Motor Velocity RPS",
          () ->
              Optional.ofNullable((AngularVelocity) motorVelocity)
                  .orElse(RotationsPerSecond.zero())
                  .in(RotationsPerSecond),
          null);
      builder.addDoubleProperty(
          "Motor Volts",
          () -> Optional.ofNullable((Voltage) voltage).orElse(Volts.zero()).in(Volts),
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

  public abstract void setInputVoltage(Voltage voltage);
}
