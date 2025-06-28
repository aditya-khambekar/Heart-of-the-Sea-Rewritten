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
    public MutAngle leftMotorPosition = Degrees.mutable(0);
    public MutAngle rightMotorPosition = Degrees.mutable(0);
    public MutAngularVelocity leftMotorSpeed = RadiansPerSecond.mutable(0);
    public MutAngularVelocity rightMotorSpeed = RadiansPerSecond.mutable(0);
    public MutTemperature leftMotorTemperature = Celsius.mutable(0);
    public MutTemperature rightMotorTemperature = Celsius.mutable(0);
    public MutCurrent leftMotorTorqueCurrent = Amps.mutable(0);
    public MutCurrent rightMotorTorqueCurrent = Amps.mutable(0);

    @Override
    public void initSendable(SendableBuilder sendableBuilder) {
      sendableBuilder.addDoubleProperty(
          "Left Motor Rotations",
          () ->
              Optional.ofNullable((Angle) leftMotorPosition).orElse(Rotations.zero()).in(Rotations),
          null);
      sendableBuilder.addDoubleProperty(
          "Right Motor Rotations",
          () ->
              Optional.ofNullable((Angle) rightMotorPosition)
                  .orElse(Rotations.zero())
                  .in(Rotations),
          null);
      sendableBuilder.addDoubleProperty(
          "Left Motor Speed Rad/s",
          () ->
              Optional.ofNullable((AngularVelocity) leftMotorSpeed)
                  .orElse(RadiansPerSecond.zero())
                  .in(Units.RadiansPerSecond),
          null);
      sendableBuilder.addDoubleProperty(
          "Right Motor Speed Rad/s",
          () ->
              Optional.ofNullable((AngularVelocity) rightMotorSpeed)
                  .orElse(RadiansPerSecond.zero())
                  .in(Units.RadiansPerSecond),
          null);
      sendableBuilder.addDoubleProperty(
          "Left Motor Celsius",
          () ->
              Optional.ofNullable((Temperature) leftMotorTemperature)
                  .orElse(Celsius.zero())
                  .in(Celsius),
          null);
      sendableBuilder.addDoubleProperty(
          "Right Motor Celsius",
          () ->
              Optional.ofNullable((Temperature) rightMotorTemperature)
                  .orElse(Celsius.zero())
                  .in(Celsius),
          null);
      sendableBuilder.addDoubleProperty(
          "Left Motor Torque Current",
          () ->
              Optional.ofNullable((Current) leftMotorTorqueCurrent)
                  .orElse(Amps.zero())
                  .in(Units.Amp),
          null);
      sendableBuilder.addDoubleProperty(
          "Right Motor Torque Current",
          () ->
              Optional.ofNullable((Current) rightMotorTorqueCurrent)
                  .orElse(Amps.zero())
                  .in(Units.Amp),
          null);
    }
  }

  public abstract void updateInputs(ElevatorIOInputs inputs);

  public abstract void zero();
}
