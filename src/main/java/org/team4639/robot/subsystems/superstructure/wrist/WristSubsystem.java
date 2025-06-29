package org.team4639.robot.subsystems.superstructure.wrist;

import static edu.wpi.first.units.Units.*;

import au.grapplerobotics.LaserCan;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.Dimensionless;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import org.team4639.lib.io.sensor.lasercan.LaserCanIO;
import org.team4639.robot.subsystems.superstructure.wrist.io.WristIO;

public class WristSubsystem extends SubsystemBase {
  WristIO.WristIOInputs wristIOInputs;
  org.team4639.lib.io.sensor.lasercan.LaserCanIO.LaserCanIOInputs laserCanIOInputs;
  WristIO WristIO;
  LaserCanIO LaserCanIO;

  public WristSubsystem(WristIO WristIO, LaserCanIO laserCanIO) {
    wristIOInputs = new WristIO.WristIOInputs();
    laserCanIOInputs = new LaserCanIO.LaserCanIOInputs();
    this.WristIO = WristIO;
    this.LaserCanIO = laserCanIO;
  }

  @Override
  public void periodic() {
    WristIO.updateInputs(wristIOInputs);
    LaserCanIO.updateInputs(laserCanIOInputs);
  }

  public void wristForward() {
    WristIO.setDutyCycleOutput(Percent.of(10));
  }

  public void wristBack() {
    WristIO.setDutyCycleOutput(Percent.of(-10));
  }

  public void setWristSetpoint(Rotation2d setpoint) {
    WristIO.setPosition(WristConstants.RotationToPosition.convert(setpoint));
  }

  public void setWristDutyCycle(Dimensionless dutyCycle) {
    WristIO.setDutyCycleOutput(dutyCycle);
  }

  public void setWristSetpoint(Angle setpoint) {
    WristIO.setPosition(setpoint);
  }

  public Rotation2d getWristAngle() {
    return WristConstants.RotationToPosition.convertBackwards(wristIOInputs.motorPosition);
  }

  public boolean hasCoral() {
    return laserCanIOInputs.measurement.in(Millimeter) < 20
        && laserCanIOInputs.status == LaserCan.LASERCAN_STATUS_VALID_MEASUREMENT;
  }

  public boolean doesNotHaveCoral() {
    return !hasCoral();
  }

  public boolean hasIntakedAlgae() {
    return wristIOInputs.motorCurrent.in(Amps) > WristConstants.ALGAE_CURRENT.in(Amps);
  }
}
