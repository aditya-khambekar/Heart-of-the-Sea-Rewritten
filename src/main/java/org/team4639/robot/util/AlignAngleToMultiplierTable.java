package org.team4639.robot.util;

import edu.wpi.first.math.interpolation.InterpolatingDoubleTreeMap;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

public class AlignAngleToMultiplierTable extends InterpolatingDoubleTreeMap {
  private static final AlignAngleToMultiplierTable instance = new AlignAngleToMultiplierTable();

  public static synchronized AlignAngleToMultiplierTable getInstance() {
    return instance;
  }

  private double angleToAdd = Math.PI;
  private double INCREMENT = 0.01;
  public static final int POWER = 3001;

  public AlignAngleToMultiplierTable() {
    super();
  }

  public void periodic() {
    if (angleToAdd > 0) {
      SmartDashboard.putNumber("Align memo angle", angleToAdd);
      this.put(angleToAdd, Math.pow(Math.cos(angleToAdd), POWER));
      this.put(-angleToAdd, Math.pow(Math.cos(-angleToAdd), POWER));
      angleToAdd -= INCREMENT;
      this.put(angleToAdd, Math.pow(Math.cos(angleToAdd), POWER));
      this.put(-angleToAdd, Math.pow(Math.cos(-angleToAdd), POWER));
      angleToAdd -= INCREMENT;
      this.put(angleToAdd, Math.pow(Math.cos(angleToAdd), POWER));
      this.put(-angleToAdd, Math.pow(Math.cos(-angleToAdd), POWER));
      angleToAdd -= INCREMENT;
      this.put(angleToAdd, Math.pow(Math.cos(angleToAdd), POWER));
      this.put(-angleToAdd, Math.pow(Math.cos(-angleToAdd), POWER));
      angleToAdd -= INCREMENT;
      this.put(angleToAdd, Math.pow(Math.cos(angleToAdd), POWER));
      this.put(-angleToAdd, Math.pow(Math.cos(-angleToAdd), POWER));
      angleToAdd -= INCREMENT;
    }
  }
}
