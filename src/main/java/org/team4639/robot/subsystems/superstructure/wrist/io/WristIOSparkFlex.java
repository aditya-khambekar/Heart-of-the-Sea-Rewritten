package org.team4639.robot.subsystems.superstructure.wrist.io;

import static edu.wpi.first.units.Units.*;

import com.revrobotics.spark.SparkFlex;
import com.revrobotics.spark.SparkLowLevel;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.Dimensionless;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

public class WristIOSparkFlex extends WristIO {
  SparkFlex sparkFlex;
  ProfiledPIDController wristPIDController;

  public WristIOSparkFlex(int ID) {
    sparkFlex = new SparkFlex(ID, SparkLowLevel.MotorType.kBrushless);

    wristPIDController =
        new ProfiledPIDController(40, 0, 0, new TrapezoidProfile.Constraints(60, 20));

    SmartDashboard.putData("Wrist PID Controller", wristPIDController);
  }

  @Override
  public void updateInputs(WristIOInputs inputs) {
    inputs.motorCurrent = Amps.of(sparkFlex.getOutputCurrent());
    inputs.motorPosition = Rotations.of(sparkFlex.getAbsoluteEncoder().getPosition());
    inputs.motorTemperature = Celsius.of(sparkFlex.getMotorTemperature());
    inputs.motorVelocity =
        RotationsPerSecond.of(sparkFlex.getAbsoluteEncoder().getVelocity() / 60.);
  }

  @Override
  public void setDutyCycleOutput(Dimensionless percent) {
    sparkFlex.set(percent.in(Value));
  }

  @Override
  public void setPosition(Angle position) {
    wristPIDController.setGoal(position.in(Rotations));
    sparkFlex.set(wristPIDController.calculate(sparkFlex.getAbsoluteEncoder().getPosition()));
  }
}
