package org.team4639.robot.subsystems.superstructure.elevator.io;

import static edu.wpi.first.units.Units.*;
import static org.team4639.robot.subsystems.superstructure.elevator.ElevatorConstants.*;

import com.ctre.phoenix6.controls.CoastOut;
import com.ctre.phoenix6.controls.ControlRequest;
import com.ctre.phoenix6.controls.NeutralOut;
import edu.wpi.first.math.controller.ElevatorFeedforward;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.math.system.plant.LinearSystemId;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.units.measure.Dimensionless;
import edu.wpi.first.units.measure.Voltage;
import edu.wpi.first.wpilibj.simulation.ElevatorSim;
import org.team4639.lib.io.motor.MotorIOTalonFX;
import org.team4639.lib.unit.Units2;

public class ElevatorIOTalonFXSim extends ElevatorIO {
  private final ElevatorFeedforward elevatorFeedforward;
  private final ProfiledPIDController elevatorPID;
  private final ElevatorSim elevatorSim;
  MotorIOTalonFX.ControlRequestGetter requestGetter;

  public ElevatorIOTalonFXSim(int leftMotorID, int rightMotorID) {
    elevatorPID = new ProfiledPIDController(1, 0, 0, new TrapezoidProfile.Constraints(50, 200));
    elevatorFeedforward = new ElevatorFeedforward(0, 0.63195475, 0.085, 0);
    elevatorSim =
        new ElevatorSim(
            LinearSystemId.createElevatorSystem(DCMotor.getKrakenX60(2), 10, 0.0762, 10),
            DCMotor.getKrakenX60(2),
            Units2.inchesToMeters.convert(15),
            Units2.inchesToMeters.convert(90),
            true,
            Units2.inchesToMeters.convert(15));
  }

  @Override
  public void updateInputs(ElevatorIOInputs inputs) {
    //    inputs.leftMotorPosition.mut_replace(leftTalon.getPosition().getValue());
    //    inputs.rightMotorPosition.mut_replace(rightTalon.getPosition().getValue());
    //    inputs.leftMotorSpeed.mut_replace(leftTalon.getVelocity().getValue());
    //    inputs.rightMotorSpeed.mut_replace(rightTalon.getVelocity().getValue());
    //    inputs.leftMotorTemperature.mut_replace(leftTalon.getDeviceTemp().getValue());
    //    inputs.rightMotorTemperature.mut_replace(rightTalon.getDeviceTemp().getValue());
    //    inputs.leftMotorTorqueCurrent.mut_replace(leftTalon.getTorqueCurrent().getValue());
    //    inputs.rightMotorTorqueCurrent.mut_replace(rightTalon.getTorqueCurrent().getValue());
  }

  private void setControl(ControlRequest request) {}

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
    elevatorPID.setGoal(mechanismPosition.in(Rotations));
    double output =
        elevatorPID.calculate(
                heightToRotations.convert(Meters.of(elevatorSim.getPositionMeters())).in(Rotations))
            + elevatorFeedforward.calculate(elevatorPID.getSetpoint().velocity);
    elevatorSim.update(0.02);
  }

  @Override
  public void setVelocitySetpoint(AngularVelocity mechanismVelocity) {
    setControl(requestGetter.getVelocityRequest(mechanismVelocity));
  }

  @Override
  public void setPositionSetpoint(Angle mechanismPosition) {
    setControl(requestGetter.getPositionRequest(mechanismPosition));
  }

  public void zero() {}
}
