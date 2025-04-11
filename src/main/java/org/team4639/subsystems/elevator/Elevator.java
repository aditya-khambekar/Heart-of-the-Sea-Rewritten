package org.team4639.subsystems.elevator;

import static org.team4639.constants.IDs.*;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.util.sendable.Sendable;
import edu.wpi.first.util.sendable.SendableBuilder;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import org.littletonrobotics.junction.Logger;
import org.littletonrobotics.junction.mechanism.LoggedMechanism2d;
import org.littletonrobotics.junction.mechanism.LoggedMechanismLigament2d;
import org.littletonrobotics.junction.mechanism.LoggedMechanismRoot2d;
import org.team4639._lib.subsystem.RSSubsystem;
import org.team4639.subsystems.elevator.ElevatorIO.ElevatorIOInputs;

public class Elevator extends RSSubsystem implements Sendable {
  public double setpointEncoderValue;

  private ElevatorIOInputs inputs;
  private final ElevatorIO io;

  private final LoggedMechanism2d elevatorView;
  private final LoggedMechanismRoot2d elevatorViewRoot;
  private final LoggedMechanismLigament2d elevatorViewLigament;

  public Elevator(ElevatorIO io) {
    SmartDashboard.putData("Elevator Tuning", this);

    this.io = io;
    this.inputs = new ElevatorIOInputsAutoLogged();

    this.elevatorView = new LoggedMechanism2d(1, 3);
    this.elevatorViewRoot = elevatorView.getRoot("Elevator View", 0.5, 0);
    this.elevatorViewLigament =
        elevatorViewRoot.append(new LoggedMechanismLigament2d("Elevator View Ligament", 0, 90));
    SmartDashboard.putData("Elevator View", elevatorView);
  }

  @Override
  public void periodic() {
    super.periodic();
    io.updateInputs(inputs);
    Logger.recordOutput("Elevator/Speed", inputs.encoderSpeed);
    Logger.recordOutput("Elevator/Position", inputs.encoderMeasurement);
    if (getCurrentCommand() != null) {
      SmartDashboard.putString("Elevator Command", getCurrentCommand().getName());
    }
  }

  @Override
  public void initSendable(SendableBuilder builder) {
    builder.setSmartDashboardType("Elevator");

    builder.addDoubleProperty(
        "kp",
        () -> ElevatorConstants.Params.kp,
        x -> {
          ElevatorConstants.Params.kp = x;
        });
    builder.addDoubleProperty(
        "ki",
        () -> ElevatorConstants.Params.ki,
        x -> {
          ElevatorConstants.Params.ki = x;
        });
    builder.addDoubleProperty(
        "kd",
        () -> ElevatorConstants.Params.kd,
        x -> {
          ElevatorConstants.Params.kd = x;
        });
    builder.addDoubleProperty(
        "ks",
        () -> ElevatorConstants.Params.ks,
        x -> {
          ElevatorConstants.Params.ks = x;
        });
    builder.addDoubleProperty(
        "kg",
        () -> ElevatorConstants.Params.kg,
        x -> {
          ElevatorConstants.Params.kg = x;
        });
    builder.addDoubleProperty(
        "kv",
        () -> ElevatorConstants.Params.kv,
        x -> {
          ElevatorConstants.Params.kv = x;
        });
    builder.addDoubleProperty(
        "ka",
        () -> ElevatorConstants.Params.ka,
        x -> {
          ElevatorConstants.Params.ka = x;
        });
    builder.addDoubleProperty(
        "acceleration",
        () -> ElevatorConstants.Params.acceleration,
        x -> {
          ElevatorConstants.Params.acceleration = x;
        });
    builder.addDoubleProperty(
        "velocity",
        () -> ElevatorConstants.Params.velocity,
        x -> {
          ElevatorConstants.Params.velocity = x;
        });
  }

  public Command runToSetpoint(double setpointEncoderValue) {
    this.setpointEncoderValue = setpointEncoderValue;
    elevatorViewLigament.setLength(
        ElevatorConstants.ProportionToPosition.convertBackwards(setpointEncoderValue) * 3);
    SmartDashboard.putData("Elevator View", elevatorView);
    return run(() -> io.setMotionMagicPosition(setpointEncoderValue));
  }

  public Command runVelocity(double velocityRPS) {
    return run(() -> io.setVelocityControl(velocityRPS)).finallyDo(() -> io.setVelocityControl(0));
  }

  public boolean atPosition() {
    return MathUtil.isNear(
        setpointEncoderValue, inputs.encoderMeasurement, ElevatorConstants.ELEVATOR_TOLERANCE);
  }
}
