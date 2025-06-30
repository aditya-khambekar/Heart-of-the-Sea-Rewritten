package org.team4639.robot.subsystems.superstructure.wrist.io;

import static edu.wpi.first.units.Units.*;

import edu.wpi.first.math.controller.ArmFeedforward;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.math.system.plant.LinearSystemId;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.Dimensionless;
import edu.wpi.first.util.sendable.Sendable;
import edu.wpi.first.util.sendable.SendableBuilder;
import edu.wpi.first.wpilibj.simulation.SingleJointedArmSim;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import org.team4639.robot.subsystems.superstructure.wrist.WristConstants;

public class WristIOSim extends WristIO {
  SingleJointedArmSim pivotSim;
  ProfiledPIDController wristPIDController;
  ArmFeedforward feedforward;

  public WristIOSim(int ID) {
    pivotSim =
        new SingleJointedArmSim(
            LinearSystemId.createSingleJointedArmSystem(
                DCMotor.getNEO(1), SingleJointedArmSim.estimateMOI(0.419, 2.22), 25.6),
            DCMotor.getNEO(1),
            25.6,
            0.419,
            WristConstants.IDLE_ROTATION.getRadians(),
            WristConstants.MAX_ROTATION.getRadians(),
            false,
            WristConstants.IDLE_ROTATION.getRadians());

    feedforward = new ArmFeedforward(0, 0, 0.5);

    wristPIDController =
        new ProfiledPIDController(3, 0, 0, new TrapezoidProfile.Constraints(60, 20));

    SmartDashboard.putData("Wrist PID Controller", wristPIDController);
    SmartDashboard.putData(
        "Arm Feedforward",
        new Sendable() {

          @Override
          public void initSendable(SendableBuilder builder) {
            builder.setSmartDashboardType("Arm Feedforward");
            builder.addDoubleProperty("ks", feedforward::getKs, feedforward::setKs);
            builder.addDoubleProperty("kg", feedforward::getKg, feedforward::setKg);
            builder.addDoubleProperty("ka", feedforward::getKa, feedforward::setKa);
            builder.addDoubleProperty("kv", feedforward::getKv, feedforward::setKv);
          }
        });
  }

  @Override
  public void updateInputs(WristIOInputs inputs) {
    inputs.motorCurrent.mut_replace(Amps.zero());
    inputs.motorPosition.mut_replace(
        WristConstants.RotationToPosition.convert(Rotation2d.fromRadians(pivotSim.getAngleRads())));
    inputs.motorTemperature.mut_replace(Celsius.zero());
    inputs.motorVelocity.mut_replace(
        Rotations.per(Minute)
            .of(
                WristConstants.RotationToPosition.convert(
                        Rotation2d.fromRadians(pivotSim.getVelocityRadPerSec()))
                    .in(Rotations)));
  }

  @Override
  public void setDutyCycleOutput(Dimensionless percent) {
    pivotSim.setInputVoltage(12 * percent.in(Value));
    pivotSim.update(0.02);
  }

  @Override
  public void setPosition(Angle position) {
    wristPIDController.setGoal(position.in(Rotations));
    pivotSim.setInputVoltage(
        -wristPIDController.calculate(
                WristConstants.RotationToPosition.convert(
                        Rotation2d.fromRadians(pivotSim.getAngleRads()))
                    .in(Rotations))
            - feedforward.calculate(
                wristPIDController.getSetpoint().position,
                wristPIDController.getSetpoint().velocity));
    pivotSim.update(0.02);
  }
}
