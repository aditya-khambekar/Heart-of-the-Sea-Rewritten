package org.team4639.robot.subsystems.superstructure.wrist;

import static edu.wpi.first.units.Units.Percent;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.Dimensionless;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import org.team4639.robot.subsystems.superstructure.wrist.io.WristIO;

public class WristSubsystem extends SubsystemBase {
  WristIO.WristIOInputs wristIOInputs;
  WristIO io;

  public WristSubsystem(WristIO io) {
    wristIOInputs = new WristIO.WristIOInputs();
    this.io = io;
  }

  @Override
  public void periodic() {
    io.updateInputs(wristIOInputs);
  }

  public void wristForward() {
    io.setDutyCycleOutput(Percent.of(10));
  }

  public void wristBack() {
    io.setDutyCycleOutput(Percent.of(-10));
  }

  public void setWristSetpoint(Rotation2d setpoint) {
    io.setPosition(WristConstants.RotationToPosition.convert(setpoint));
  }

  public void setWristDutyCycle(Dimensionless dutyCycle) {
    io.setDutyCycleOutput(dutyCycle);
  }

  public void setWristSetpoint(Angle setpoint) {
    io.setPosition(setpoint);
  }

  public Rotation2d getWristAngle() {
    return WristConstants.RotationToPosition.convertBackwards(wristIOInputs.motorPosition);
  }
}
