package org.team4639.robot.subsystems.superstructure.subsystems.wrist.io;

import static edu.wpi.first.units.Units.*;

import com.revrobotics.spark.SparkBase;
import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkBase.ResetMode;
import com.revrobotics.spark.SparkFlex;
import com.revrobotics.spark.SparkLowLevel;
import com.revrobotics.spark.config.AbsoluteEncoderConfig;
import com.revrobotics.spark.config.ClosedLoopConfig;
import com.revrobotics.spark.config.SparkFlexConfig;
import edu.wpi.first.math.controller.ArmFeedforward;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.Dimensionless;
import edu.wpi.first.units.measure.Voltage;

public class WristIOREVPID extends WristIO {
  SparkFlex sparkFlex;

  ArmFeedforward withoutCoral;
  ArmFeedforward withCoral;

  private double lastRequestedVolts = 0;

  public WristIOREVPID(int ID) {
    sparkFlex = new SparkFlex(ID, SparkLowLevel.MotorType.kBrushless);

    var PIDConfig = new SparkFlexConfig().closedLoop;
    PIDConfig.feedbackSensor(ClosedLoopConfig.FeedbackSensor.kPrimaryEncoder)
        .p(48.0 / 12)
        .outputRange(-1, 1)
        .maxMotion
        .maxAcceleration(180)
        .maxVelocity(60);

    sparkFlex.configure(
        new SparkFlexConfig().apply(new AbsoluteEncoderConfig().zeroOffset(0.46)).apply(PIDConfig),
        ResetMode.kNoResetSafeParameters,
        PersistMode.kPersistParameters);

    withoutCoral = new ArmFeedforward(0, 0.22, 0);
    withCoral = new ArmFeedforward(0, 0.28, 0);
  }

  @Override
  public void updateInputs(WristIOInputs inputs) {
    inputs.motorCurrent.mut_replace(Amps.of(sparkFlex.getOutputCurrent()));
    inputs.motorPosition.mut_replace(Rotations.of(sparkFlex.getAbsoluteEncoder().getPosition()));
    inputs.motorTemperature.mut_replace(Celsius.of(sparkFlex.getMotorTemperature()));
    inputs.motorVelocity.mut_replace(
        RotationsPerSecond.of(sparkFlex.getAbsoluteEncoder().getVelocity() / 60.));
    inputs.busVoltage.mut_replace(Volts.of(sparkFlex.getBusVoltage()));
    inputs.requestedVoltage.mut_replace(Volts.of(lastRequestedVolts));
  }

  @Override
  public void setDutyCycleOutput(Dimensionless percent) {
    sparkFlex.set(percent.in(Value));
    lastRequestedVolts = percent.in(Value) * 12;
  }

  @Override
  public void setPosition(Angle position) {
    sparkFlex
        .getClosedLoopController()
        .setReference(position.in(Rotations), SparkBase.ControlType.kMAXMotionPositionControl);
  }

  @Override
  public void setVoltage(Voltage voltage) {
    sparkFlex.setVoltage(voltage);
    lastRequestedVolts = voltage.in(Volts);
  }
}
