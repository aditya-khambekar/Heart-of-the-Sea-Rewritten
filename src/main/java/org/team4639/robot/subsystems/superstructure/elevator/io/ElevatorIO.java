package org.team4639.robot.subsystems.superstructure.elevator.io;

import static edu.wpi.first.units.Units.*;

import edu.wpi.first.units.Units;
import edu.wpi.first.units.measure.*;
import edu.wpi.first.util.sendable.Sendable;
import edu.wpi.first.util.sendable.SendableBuilder;
import java.util.Optional;
import org.littletonrobotics.junction.AutoLog;

public abstract class ElevatorIO {
  public abstract void setNeutralSetpoint();

  public abstract void setCoastSetpoint();

  public abstract void setVoltageSetpoint(Voltage voltage);

  public abstract void setDutyCycleSetpoint(Dimensionless percent);

  public abstract void setMotionMagicSetpoint(Angle mechanismPosition);

  public abstract void setVelocitySetpoint(AngularVelocity mechanismVelocity);

  public abstract void setPositionSetpoint(Angle mechanismPosition);

  @AutoLog
  public static class ElevatorIOInputs implements Sendable {
    Angle leftMotorPosition;
    Angle rightMotorPosition;
    AngularVelocity leftMotorSpeed;
    AngularVelocity rightMotorSpeed;
    Temperature leftMotorTemperature;
    Temperature rightMotorTemperature;
    Current leftMotorTorqueCurrent;
    Current rightMotorTorqueCurrent;

    @Override
    public void initSendable(SendableBuilder sendableBuilder) {
      sendableBuilder.addDoubleProperty(
          "Left Motor Rotations",
          () -> Optional.ofNullable(leftMotorPosition).orElse(Rotations.zero()).in(Rotations),
          null);
      sendableBuilder.addDoubleProperty(
          "Right Motor Rotations",
          () -> Optional.ofNullable(rightMotorPosition).orElse(Rotations.zero()).in(Rotations),
          null);
      sendableBuilder.addDoubleProperty(
          "Left Motor Speed Rad/s",
          () ->
              Optional.ofNullable(leftMotorSpeed)
                  .orElse(RadiansPerSecond.zero())
                  .in(Units.RadiansPerSecond),
          null);
      sendableBuilder.addDoubleProperty(
          "Right Motor Speed Rad/s",
          () ->
              Optional.ofNullable(rightMotorSpeed)
                  .orElse(RadiansPerSecond.zero())
                  .in(Units.RadiansPerSecond),
          null);
      sendableBuilder.addDoubleProperty(
          "Left Motor Celsius",
          () -> Optional.ofNullable(leftMotorTemperature).orElse(Celsius.zero()).in(Celsius),
          null);
      sendableBuilder.addDoubleProperty(
          "Right Motor Celsius",
          () -> Optional.ofNullable(rightMotorTemperature).orElse(Celsius.zero()).in(Celsius),
          null);
      sendableBuilder.addDoubleProperty(
          "Left Motor Torque Current",
          () -> Optional.ofNullable(leftMotorTorqueCurrent).orElse(Amps.zero()).in(Units.Amp),
          null);
      sendableBuilder.addDoubleProperty(
          "Right Motor Torque Current",
          () -> Optional.ofNullable(rightMotorTorqueCurrent).orElse(Amps.zero()).in(Units.Amp),
          null);
    }
  }

  public abstract void updateInputs(ElevatorIOInputs inputs);
}
