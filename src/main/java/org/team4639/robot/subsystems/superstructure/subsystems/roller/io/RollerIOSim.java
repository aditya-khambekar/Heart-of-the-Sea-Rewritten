package org.team4639.robot.subsystems.superstructure.subsystems.roller.io;

import static edu.wpi.first.units.Units.Minute;
import static edu.wpi.first.units.Units.Rotations;
import static edu.wpi.first.units.Units.Value;
import static edu.wpi.first.units.Units.Volts;

import com.revrobotics.sim.SparkFlexSim;
import com.revrobotics.spark.SparkFlex;
import com.revrobotics.spark.SparkLowLevel;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.math.system.plant.LinearSystemId;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.units.measure.Dimensionless;
import edu.wpi.first.units.measure.Voltage;
import edu.wpi.first.wpilibj.simulation.FlywheelSim;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import org.team4639.lib.unit.Units2;

public class RollerIOSim extends RollerIO {
  private final SparkFlexSim spark;
  private final FlywheelSim simPlant;
  private final PIDController simRollerPID;

  public RollerIOSim(int ID) {
    spark =
        new SparkFlexSim(
            new SparkFlex(ID, SparkLowLevel.MotorType.kBrushless), DCMotor.getNeoVortex(1));
    simPlant =
        new FlywheelSim(
            LinearSystemId.createFlywheelSystem(
                DCMotor.getNeoVortex(1),
                0.5 * Units2.poundsToKilograms.convert(0.269) * 12.25,
                1.0),
            DCMotor.getNeoVortex(1));
    simRollerPID = new PIDController(0, 0, 0);

    SmartDashboard.putData("Roller PID", simRollerPID);
  }

  @Override
  public void updateInputs(RollerIOInputs inputs) {
    inputs.motorVelocity.mut_replace(simPlant.getAngularVelocityRPM(), Rotations.per(Minute));

    simPlant.update(0.02);
  }

  @Override
  public void setDutyCycleOutput(Dimensionless percent) {
    simPlant.setInputVoltage(12 * percent.in(Value));
  }

  @Override
  public void setVelocity(AngularVelocity velocity) {
    simRollerPID.setSetpoint(velocity.in(Rotations.per(Minute)));
    simPlant.setInputVoltage(
        -simRollerPID.calculate(spark.getVelocity()) + 0 * velocity.in(Rotations.per(Minute)));
  }

  @Override
  public void setInputVoltage(Voltage voltage) {
    simPlant.setInputVoltage(voltage.in(Volts));
  }
}
