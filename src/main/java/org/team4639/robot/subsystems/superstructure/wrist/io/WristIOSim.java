package org.team4639.robot.subsystems.superstructure.wrist.io;

import static edu.wpi.first.units.Units.*;

import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.math.system.plant.LinearSystemId;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.Dimensionless;
import edu.wpi.first.wpilibj.simulation.SingleJointedArmSim;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import org.team4639.robot.subsystems.superstructure.wrist.WristConstants;

public class WristIOSim extends WristIO {
  SingleJointedArmSim pivotSim;
  ProfiledPIDController wristPIDController;

  public WristIOSim(int ID) {
    pivotSim =
        new SingleJointedArmSim(
            LinearSystemId.createSingleJointedArmSystem(
                DCMotor.getNEO(1), SingleJointedArmSim.estimateMOI(0.419, 2.22), 25.6),
            DCMotor.getNEO(1),
            25.6,
            0.419,
            WristConstants.MAX_ROTATION.getRadians(),
            WristConstants.IDLE_ROTATION.getRadians(),
            false,
            WristConstants.IDLE_ROTATION.getRadians());

    wristPIDController =
        new ProfiledPIDController(40, 0, 0, new TrapezoidProfile.Constraints(60, 20));

    SmartDashboard.putData("Wrist PID Controller", wristPIDController);
  }

  @Override
  public void updateInputs(WristIOInputs inputs) {
    //    inputs.motorCurrent.mut_replace(Amps.of(sparkFlex.getOutputCurrent()));
    //
    // inputs.motorPosition.mut_replace(Rotations.of(sparkFlex.getAbsoluteEncoder().getPosition()));
    //    inputs.motorTemperature.mut_replace(Celsius.of(sparkFlex.getMotorTemperature()));
    //    inputs.motorVelocity.mut_replace(
    //            RotationsPerSecond.of(sparkFlex.getAbsoluteEncoder().getVelocity() / 60.));
  }

  @Override
  public void setDutyCycleOutput(Dimensionless percent) {}

  @Override
  public void setPosition(Angle position) {
    wristPIDController.setGoal(position.in(Rotations));
    pivotSim.setInputVoltage(
        wristPIDController.calculate(
            WristConstants.RotationToPosition.convert(
                    Rotation2d.fromRadians(pivotSim.getAngleRads()))
                .in(Rotations)));
    pivotSim.update(0.02);
  }
}
