package org.team4639.robot.subsystems.elevator;

import com.ctre.phoenix6.configs.CurrentLimitsConfigs;
import com.ctre.phoenix6.configs.MotionMagicConfigs;
import com.ctre.phoenix6.configs.Slot0Configs;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.Follower;
import com.ctre.phoenix6.controls.MotionMagicVoltage;
import com.ctre.phoenix6.controls.VelocityVoltage;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.GravityTypeValue;
import com.ctre.phoenix6.signals.NeutralModeValue;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.math.system.plant.LinearSystemId;
import edu.wpi.first.wpilibj.simulation.DCMotorSim;
import org.team4639.robot.constants.IDs;

public class ElevatorIOSim implements ElevatorIO {
  private final TalonFX leftMotor;
  private final TalonFX rightMotor;
  private final MotionMagicVoltage control;
  private final DCMotorSim sim;

  public ElevatorIOSim(TalonFX leftMotor, TalonFX rightMotor) {
    this.leftMotor = leftMotor;
    this.rightMotor = rightMotor;
    leftMotor.setNeutralMode(NeutralModeValue.Brake);
    rightMotor.setNeutralMode(NeutralModeValue.Brake);

    rightMotor.setControl(new Follower(IDs.ELEVATOR_LEFT, true));

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
    control = new MotionMagicVoltage(ElevatorConstants.Setpoints.ELEVATOR_LOWEST_PROPORTION);

    sim =
        new DCMotorSim(
            LinearSystemId.createElevatorSystem(DCMotor.getKrakenX60(2), 25, 0.25, 1 / 13.44),
            DCMotor.getKrakenX60(2),
            0.0,
            0.0);
  }

  @Override
  public void updateInputs(ElevatorIOInputs inputs) {
    inputs.encoderMeasurement = sim.getAngularPositionRotations();
    inputs.encoderSpeed = sim.getAngularVelocityRPM() / 60.0;
  }

  public void setMotionMagicPosition(double setpointEncoder) {
    if (control.Position != setpointEncoder) {
      control.Position = setpointEncoder;
    }

    leftMotor.setControl(control);
    sim.setInputVoltage(leftMotor.getSimState().getMotorVoltage());
  }

  public void setVelocityControlRPS(double velocityRPS) {
    leftMotor.setControl(new VelocityVoltage(velocityRPS));
    sim.setInputVoltage(leftMotor.getSimState().getMotorVoltage());
  }
}
