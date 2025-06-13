package org.team4639.robot.subsystems.superstructure.elevator.io;

import com.ctre.phoenix6.controls.CoastOut;
import com.ctre.phoenix6.controls.ControlRequest;
import com.ctre.phoenix6.controls.Follower;
import com.ctre.phoenix6.controls.NeutralOut;
import com.ctre.phoenix6.hardware.TalonFX;
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
    leftTalon = new TalonFX(leftMotorID);
    rightTalon = new TalonFX(rightMotorID);

    rightTalon.setControl(new Follower(leftMotorID, true));
    requestGetter = new MotorIOTalonFX.ControlRequestGetter();
  }

  @Override
  public void updateInputs(ElevatorIOInputs inputs) {
    inputs.leftMotorPosition = leftTalon.getPosition().getValue();
    inputs.rightMotorPosition = rightTalon.getPosition().getValue();
    inputs.leftMotorSpeed = leftTalon.getVelocity().getValue();
    inputs.rightMotorSpeed = rightTalon.getVelocity().getValue();
    inputs.leftMotorTemperature = leftTalon.getDeviceTemp().getValue();
    inputs.rightMotorTemperature = rightTalon.getDeviceTemp().getValue();
    inputs.leftMotorTorqueCurrent = leftTalon.getTorqueCurrent().getValue();
    inputs.rightMotorTorqueCurrent = rightTalon.getTorqueCurrent().getValue();
  }

  private void setControl(ControlRequest request) {
    leftTalon.setControl(request);
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
    setControl(requestGetter.getMotionMagicRequest(mechanismPosition));
  }

  @Override
  public void setVelocitySetpoint(AngularVelocity mechanismVelocity) {
    setControl(requestGetter.getVelocityRequest(mechanismVelocity));
  }

  @Override
  public void setPositionSetpoint(Angle mechanismPosition) {
    setControl(requestGetter.getPositionRequest(mechanismPosition));
  }
}
