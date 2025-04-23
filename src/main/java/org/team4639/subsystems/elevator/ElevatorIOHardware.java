package org.team4639.subsystems.elevator;

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

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

import org.team4639.constants.IDs;
import org.team4639.subsystems.elevator.ElevatorConstants;

public class ElevatorIOHardware implements ElevatorIO {
  private final TalonFX leftMotor;
  private final TalonFX rightMotor;
  private final MotionMagicVoltage control;

  public ElevatorIOHardware(TalonFX leftMotor, TalonFX rightMotor) {
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
            .withCurrentLimits(
                new CurrentLimitsConfigs()
                    .withStatorCurrentLimit(ElevatorConstants.statorCurrentLimit));

    leftMotor.getConfigurator().apply(configuration);
    control = new MotionMagicVoltage(ElevatorConstants.Setpoints.ELEVATOR_LOWEST_PROPORTION);
  }

  @Override
  public void updateInputs(ElevatorIOInputs inputs) {
    inputs.encoderMeasurement = leftMotor.getPosition().getValueAsDouble();
    inputs.encoderSpeed = leftMotor.getVelocity().getValueAsDouble();

    SmartDashboard.putNumber("elevator/right motor temp", rightMotor.getDeviceTemp().getValueAsDouble());
    SmartDashboard.putNumber("elevator/left motor temp", leftMotor.getDeviceTemp().getValueAsDouble());
  }

  public void setMotionMagicPosition(double setpointEncoder) {
    if (control.Position != setpointEncoder) {
      control.Position = setpointEncoder;
    }

    leftMotor.setControl(control);
  }

  public void setVelocityControlRPS(double velocityRPS) {
    leftMotor.setControl(new VelocityVoltage(velocityRPS));
  }
}
