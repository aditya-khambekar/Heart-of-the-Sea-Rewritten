package org.team4639.subsystems.drive;

import com.pathplanner.lib.config.ModuleConfig;
import com.pathplanner.lib.config.RobotConfig;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.math.util.Units;
import org.team4639.subsystems.drive.generated.TunerConstants;

public class RobotConfigGenerator {
  private static RobotConfig robotConfig =
      new RobotConfig(
          Units.lbsToKilograms(135),
          3.97,
          new ModuleConfig(
              edu.wpi.first.units.Units.Inches.of(1.94),
              TunerConstants.kSpeedAt12Volts,
              1.9,
              DCMotor.getKrakenX60(1),
              edu.wpi.first.units.Units.Amps.of(DriveConstants.driverStatorCurrentLimit),
              1));

  public static RobotConfig getRobotConfig() {
    return robotConfig;
  }
}
