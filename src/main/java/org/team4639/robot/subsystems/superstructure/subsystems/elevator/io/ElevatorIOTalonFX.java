package org.team4639.robot.subsystems.superstructure.subsystems.elevator.io;

import static org.team4639.robot.subsystems.superstructure.subsystems.elevator.ElevatorConstants.*;

import com.ctre.phoenix6.configs.CurrentLimitsConfigs;
import com.ctre.phoenix6.configs.MotionMagicConfigs;
import com.ctre.phoenix6.configs.Slot0Configs;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.CoastOut;
import com.ctre.phoenix6.controls.ControlRequest;
import com.ctre.phoenix6.controls.Follower;
import com.ctre.phoenix6.controls.NeutralOut;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.GravityTypeValue;
import com.ctre.phoenix6.signals.NeutralModeValue;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.units.measure.Dimensionless;
import edu.wpi.first.units.measure.Voltage;
import org.team4639.lib.io.motor.MotorIOTalonFX;

public class ElevatorIOTalonFX extends ElevatorIO {
  TalonFX leftTalon;
  TalonFX rightTalon;
  MotorIOTalonFX.ControlRequestGetter requestGetter;

  public ElevatorIOTalonFX(int leftMotorID, int rightMotorID) {
    leftTalon = new TalonFX(leftMotorID, "MainCANivore");
    rightTalon = new TalonFX(rightMotorID, "MainCANivore");

    TalonFXConfiguration configuration =
        new TalonFXConfiguration()
            .withSlot0(
                new Slot0Configs()
                    .withKP(elevatorKp.get() / 2.)
                    .withKI(elevatorKi.get())
                    .withKD(elevatorKd.get())
                    .withKA(elevatorKa.get())
                    .withKS(elevatorKs.get())
                    .withKV(elevatorKv.get())
                    .withKG(elevatorKg.get())
                    .withGravityType(GravityTypeValue.Elevator_Static))
            .withMotionMagic(
                new MotionMagicConfigs()
                    .withMotionMagicAcceleration(elevatorAcceleration.get())
                    .withMotionMagicCruiseVelocity(elevatorVelocity.get()));
    configuration.withCurrentLimits(new CurrentLimitsConfigs().withStatorCurrentLimit(80));

    leftTalon.getConfigurator().apply(configuration);
    rightTalon.getConfigurator().apply(configuration);

    leftTalon.setNeutralMode(NeutralModeValue.Brake);
    rightTalon.setNeutralMode(NeutralModeValue.Brake);

    // set left motor to follow right motor
    leftTalon.setControl(new Follower(rightMotorID, true));
    requestGetter = new MotorIOTalonFX.ControlRequestGetter();

    leftTalon.setPosition(0);
    rightTalon.setPosition(0);
  }

  @Override
  public void updateInputs(ElevatorIOInputs inputs) {
    inputs.leftMotorPosition.mut_replace(leftTalon.getPosition().getValue());
    inputs.rightMotorPosition.mut_replace(rightTalon.getPosition().getValue());
    inputs.leftMotorSpeed.mut_replace(leftTalon.getVelocity().getValue());
    inputs.rightMotorSpeed.mut_replace(rightTalon.getVelocity().getValue());
    inputs.leftMotorTemperature.mut_replace(leftTalon.getDeviceTemp().getValue());
    inputs.rightMotorTemperature.mut_replace(rightTalon.getDeviceTemp().getValue());
    inputs.leftMotorTorqueCurrent.mut_replace(leftTalon.getTorqueCurrent().getValue());
    inputs.rightMotorTorqueCurrent.mut_replace(rightTalon.getTorqueCurrent().getValue());
  }

  private void setControl(ControlRequest request) {
    rightTalon.setControl(request);
  }

  @Override
  public void setNeutralSetpoint() {
    setControl(new NeutralOut());
  }

  @Override
  public void setCoastSetpoint() {
    setControl(new CoastOut());
  }

  @Override
  public void setVoltageSetpoint(Voltage voltage) {
    setControl(requestGetter.getVoltageRequest(voltage));
  }

  @Override
  public void setDutyCycleSetpoint(Dimensionless percent) {
    setControl(requestGetter.getDutyCycleRequest(percent));
  }

  @Override
  public void setMotionMagicSetpoint(Angle mechanismPosition) {
    if (mechanismPosition.gte(rightTalon.getPosition().getValue())) {
      rightTalon
          .getConfigurator()
          .apply(
              new MotionMagicConfigs()
                  .withMotionMagicAcceleration(elevatorAcceleration.get())
                  .withMotionMagicCruiseVelocity(elevatorVelocity.get()));
    } else {
      rightTalon
          .getConfigurator()
          .apply(
              new MotionMagicConfigs()
                  .withMotionMagicAcceleration(elevatorAcceleration.get() * 2.0)
                  .withMotionMagicCruiseVelocity(elevatorVelocity.get()));
    }
    setControl(requestGetter.getMotionMagicRequestSlot0(mechanismPosition));
  }

  @Override
  public void setVelocitySetpoint(AngularVelocity mechanismVelocity) {
    setControl(requestGetter.getVelocityRequest(mechanismVelocity));
  }

  @Override
  public void setPositionSetpoint(Angle mechanismPosition) {
    setControl(requestGetter.getPositionRequest(mechanismPosition));
  }

  public void zero() {
    leftTalon.setPosition(0);
    rightTalon.setPosition(0);
  }
}
