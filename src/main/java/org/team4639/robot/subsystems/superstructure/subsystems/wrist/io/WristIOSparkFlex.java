package org.team4639.robot.subsystems.superstructure.subsystems.wrist.io;

import static edu.wpi.first.units.Units.*;

import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkBase.ResetMode;
import com.revrobotics.spark.SparkFlex;
import com.revrobotics.spark.SparkLowLevel;
import com.revrobotics.spark.config.AbsoluteEncoderConfig;
import com.revrobotics.spark.config.SparkFlexConfig;
import edu.wpi.first.math.controller.ArmFeedforward;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.Dimensionless;
import edu.wpi.first.units.measure.Voltage;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import org.team4639.robot.robot.Subsystems;

public class WristIOSparkFlex extends WristIO {
  SparkFlex sparkFlex;
  ProfiledPIDController wristPIDController;

  ArmFeedforward withoutCoral;
  ArmFeedforward withCoral;

  public WristIOSparkFlex(int ID) {
    sparkFlex = new SparkFlex(ID, SparkLowLevel.MotorType.kBrushless);

    wristPIDController =
        new ProfiledPIDController(48, 0, 0, new TrapezoidProfile.Constraints(60, 180));

    SmartDashboard.putData("Wrist PID Controller", wristPIDController);

    sparkFlex.configure(
        new SparkFlexConfig().apply(new AbsoluteEncoderConfig().zeroOffset(0.46)),
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
    inputs.motorVoltage.mut_replace(Volts.of(sparkFlex.getBusVoltage()));
  }

  @Override
  public void setDutyCycleOutput(Dimensionless percent) {
    sparkFlex.set(percent.in(Value));
  }

  @Override
  public void setPosition(Angle position) {
    wristPIDController.setGoal(position.in(Rotations));
    sparkFlex.setVoltage(
        (Subsystems.wrist.hasCoral() ? withCoral : withoutCoral)
                .calculate(
                    Subsystems.wrist.getWristAngle().minus(Rotation2d.fromDegrees(35)).getRadians(),
                    0)
            + wristPIDController.calculate(sparkFlex.getAbsoluteEncoder().getPosition()));
  }

  @Override
  public void setVoltage(Voltage voltage) {
    sparkFlex.setVoltage(voltage);
  }
}
