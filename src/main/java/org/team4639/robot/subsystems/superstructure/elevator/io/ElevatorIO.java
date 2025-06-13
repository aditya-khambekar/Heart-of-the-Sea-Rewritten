package org.team4639.robot.subsystems.superstructure.elevator.io;

import edu.wpi.first.units.Units;
import edu.wpi.first.units.measure.*;
import edu.wpi.first.util.sendable.Sendable;
import edu.wpi.first.util.sendable.SendableBuilder;
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
          "Left Motor Rotations", () -> leftMotorPosition.in(Units.Rotations), null);
      sendableBuilder.addDoubleProperty(
          "Right Motor Rotations", () -> rightMotorPosition.in(Units.Rotations), null);
      sendableBuilder.addDoubleProperty(
          "Left Motor Speed Rad/s", () -> leftMotorSpeed.in(Units.RadiansPerSecond), null);
      sendableBuilder.addDoubleProperty(
          "Right Motor Speed Rad/s", () -> rightMotorSpeed.in(Units.RadiansPerSecond), null);
      sendableBuilder.addDoubleProperty(
          "Left Motor Celsius", () -> leftMotorTemperature.in(Units.Celsius), null);
      sendableBuilder.addDoubleProperty(
          "Right Motor Celsius", () -> rightMotorTemperature.in(Units.Celsius), null);
      sendableBuilder.addDoubleProperty(
          "Left Motor Torque Current", () -> leftMotorTorqueCurrent.in(Units.Amp), null);
      sendableBuilder.addDoubleProperty(
          "Right Motor Torque", () -> rightMotorTorqueCurrent.in(Units.Amp), null);
    }
  }

  public abstract void updateInputs(ElevatorIOInputs inputs);
}
