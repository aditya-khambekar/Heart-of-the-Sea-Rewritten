package org.team4639.robot.subsystems;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public final class DashboardOutputs extends SubsystemBase {
  public DashboardOutputs() {
    SmartDashboard.putString("Reef Level", "NONE");
  }

  private int level = 0;

  public InstantCommand setL4() {
    return new InstantCommand(() -> level = 4);
  }

  public InstantCommand setL3() {
    return new InstantCommand(() -> level = 3);
  }

  public InstantCommand setL2() {
    return new InstantCommand(() -> level = 2);
  }

  public InstantCommand setL1() {
    return new InstantCommand(() -> level = 1);
  }

  public int upcomingReefLevel() {
    return level;
  }

  @Override
  public void periodic() {
    SmartDashboard.putNumber("Reef Level", level);
  }
}
