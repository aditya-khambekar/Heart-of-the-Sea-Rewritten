package org.team4639.robot.subsystems;

import static edu.wpi.first.units.Units.Seconds;

import edu.wpi.first.units.measure.MutTime;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import org.team4639.lib.limelight.LimelightHelpers;

public class LimelightFlash extends SubsystemBase {
  private String name;
  private MutTime timeSinceLastCall = Seconds.mutable(1000);

  public LimelightFlash(String name) {
    this.name = name;
  }

  public void flash() {
    timeSinceLastCall.mut_replace(0, Seconds);
  }

  @Override
  public void periodic() {
    timeSinceLastCall.mut_plus(0.02, Seconds);

    if (timeSinceLastCall.lte(Seconds.of(0.1))) {
      LimelightHelpers.setLEDMode_ForceOn(name);
      SmartDashboard.putBoolean("Limelight Flashbang", true);
    } else {
      LimelightHelpers.setLEDMode_ForceOff(name);
      SmartDashboard.putBoolean("Limelight Flashbang", false);
    }
  }
}
