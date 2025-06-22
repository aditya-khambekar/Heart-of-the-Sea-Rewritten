package org.team4639.robot.subsystems.superstructure.roller.io;

import static edu.wpi.first.units.Units.*;

import com.revrobotics.spark.SparkBase;
import com.revrobotics.spark.SparkFlex;
import com.revrobotics.spark.SparkLowLevel;
import com.revrobotics.spark.config.ClosedLoopConfig;
import com.revrobotics.spark.config.SparkFlexConfig;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.units.measure.Dimensionless;

public class RollerIOSparkFlex extends RollerIO {
  SparkFlex sparkFlex;

  public RollerIOSparkFlex(int ID) {
    sparkFlex = new SparkFlex(ID, SparkLowLevel.MotorType.kBrushless);

    SparkFlexConfig config = new SparkFlexConfig();

    config
        .closedLoop
        .feedbackSensor(ClosedLoopConfig.FeedbackSensor.kPrimaryEncoder)
        .p(0.1)
        .i(0)
        .d(0)
        .outputRange(-1, 1);

    sparkFlex.configure(
        config, SparkBase.ResetMode.kResetSafeParameters, SparkBase.PersistMode.kPersistParameters);
  }

  @Override
  public void updateInputs(RollerIOInputs inputs) {
    // TODO: make this better
    inputs.motorCurrent.mut_replace(Amps.of(sparkFlex.getOutputCurrent()));
    inputs.motorTemperature.mut_replace(Celsius.of(sparkFlex.getMotorTemperature()));
    inputs.motorVelocity.mut_replace(
        Rotations.per(Minute).of(sparkFlex.getAbsoluteEncoder().getVelocity()));
  }

  @Override
  public void setDutyCycleOutput(Dimensionless percent) {
    sparkFlex.set(percent.in(Value));
  }

  @Override
  public void setVelocity(AngularVelocity velocity) {
    sparkFlex
        .getClosedLoopController()
        .setReference(velocity.in(Rotations.per(Minute)), SparkBase.ControlType.kVelocity);
  }
}
