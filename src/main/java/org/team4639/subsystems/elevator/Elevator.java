package org.team4639.subsystems.elevator;

import static org.team4639.constants.IDs.*;

import com.ctre.phoenix6.configs.CurrentLimitsConfigs;
import com.ctre.phoenix6.configs.MotionMagicConfigs;
import com.ctre.phoenix6.configs.Slot0Configs;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.Follower;
import com.ctre.phoenix6.controls.MotionMagicVoltage;
import com.ctre.phoenix6.signals.GravityTypeValue;
import edu.wpi.first.util.sendable.Sendable;
import edu.wpi.first.util.sendable.SendableBuilder;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import org.littletonrobotics.junction.Logger;
import org.team4639._lib.motorcontrol.generic.NeutralMode;
import org.team4639._lib.motorcontrol.talonfx.RSTalonFX;
import org.team4639._lib.motorcontrol.talonfx.RSTalonFXTemplate;
import org.team4639._lib.subsystem.RSSubsystem;

public class Elevator extends RSSubsystem implements Sendable {
  private final RSTalonFX leftMotor;
  private final RSTalonFX rightMotor;
  private double setpointEncoderValue;

  private ElevatorIOInputsAutoLogged inputs;
  private final ElevatorIO io;

  public Elevator(ElevatorIO io) {
    leftMotor = RSTalonFXTemplate.KrakenX60(ELEVATOR_LEFT);
    rightMotor = RSTalonFXTemplate.KrakenX60(ELEVATOR_RIGHT);

    leftMotor.setNeutralMode(NeutralMode.BRAKE);
    rightMotor.setNeutralMode(NeutralMode.BRAKE);

    rightMotor.setControl(new Follower(ELEVATOR_LEFT, true));
    SmartDashboard.putData("Elevator Tuning", this);

    this.io = io;
    this.inputs = new ElevatorIOInputsAutoLogged();
    io.sendTalonInputs(
        leftMotor.getVelocity()::getValueAsDouble, leftMotor.getPosition()::getValueAsDouble);
    regenerateConfiguration();
  }

  private void regenerateConfiguration() {
    TalonFXConfiguration configuration =
        new TalonFXConfiguration()
            .withMotionMagic(
                new MotionMagicConfigs()
                    .withMotionMagicAcceleration(ElevatorConstants.Params.acceleration)
                    .withMotionMagicCruiseVelocity(ElevatorConstants.Params.velocity))
            .withSlot0(
                new Slot0Configs()
                    .withKP(ElevatorConstants.Params.kp)
                    .withKI(ElevatorConstants.Params.ki)
                    .withKD(ElevatorConstants.Params.kd)
                    .withKA(ElevatorConstants.Params.ka)
                    .withKS(ElevatorConstants.Params.ks)
                    .withKV(ElevatorConstants.Params.kv)
                    .withKG(ElevatorConstants.Params.kg)
                    .withGravityType(GravityTypeValue.Elevator_Static))
            .withCurrentLimits(new CurrentLimitsConfigs().withStatorCurrentLimit(45));

    leftMotor.getConfigurator().apply(configuration);
  }

  @Override
  public void periodic() {
    super.periodic();
    io.updateInputs(inputs);
    Logger.recordOutput("Elevator/Speed", inputs.encoderSpeed);
    Logger.recordOutput("Elevator/Position", inputs.encoderMeasurement);
  }

  @Override
  public void initSendable(SendableBuilder builder) {
    builder.setSmartDashboardType("Elevator");

    builder.addDoubleProperty(
        "kp",
        () -> ElevatorConstants.Params.kp,
        x -> {
          ElevatorConstants.Params.kp = x;
          regenerateConfiguration();
        });
    builder.addDoubleProperty(
        "ki",
        () -> ElevatorConstants.Params.ki,
        x -> {
          ElevatorConstants.Params.ki = x;
          regenerateConfiguration();
        });
    builder.addDoubleProperty(
        "kd",
        () -> ElevatorConstants.Params.kd,
        x -> {
          ElevatorConstants.Params.kd = x;
          regenerateConfiguration();
        });
    builder.addDoubleProperty(
        "ks",
        () -> ElevatorConstants.Params.ks,
        x -> {
          ElevatorConstants.Params.ks = x;
          regenerateConfiguration();
        });
    builder.addDoubleProperty(
        "kg",
        () -> ElevatorConstants.Params.kg,
        x -> {
          ElevatorConstants.Params.kg = x;
          regenerateConfiguration();
        });
    builder.addDoubleProperty(
        "kv",
        () -> ElevatorConstants.Params.kv,
        x -> {
          ElevatorConstants.Params.kv = x;
          regenerateConfiguration();
        });
    builder.addDoubleProperty(
        "ka",
        () -> ElevatorConstants.Params.ka,
        x -> {
          ElevatorConstants.Params.ka = x;
          regenerateConfiguration();
        });
    builder.addDoubleProperty(
        "acceleration",
        () -> ElevatorConstants.Params.acceleration,
        x -> {
          ElevatorConstants.Params.acceleration = x;
          regenerateConfiguration();
        });
    builder.addDoubleProperty(
        "velocity",
        () -> ElevatorConstants.Params.velocity,
        x -> {
          ElevatorConstants.Params.velocity = x;
          regenerateConfiguration();
        });
  }

  public Command runToSetpoint(double setpointEncoderValue) {
    this.setpointEncoderValue = setpointEncoderValue;
    return run(() -> leftMotor.setControl(new MotionMagicVoltage(this.setpointEncoderValue)));
  }
}
