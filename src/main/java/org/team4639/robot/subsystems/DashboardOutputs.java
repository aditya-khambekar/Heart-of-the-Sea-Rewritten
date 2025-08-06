package org.team4639.robot.subsystems;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import org.team4639.robot.constants.Controls;
import org.team4639.robot.robot.Subsystems;

import java.util.Objects;

public class DashboardOutputs{

    private static volatile DashboardOutputs instance;
    private int upcomingReefLevel = 0;

    public static synchronized DashboardOutputs getInstance(){
        return Objects.requireNonNullElseGet(instance, DashboardOutputs::new);
    }

  private DashboardOutputs() {
    SmartDashboard.putString("Reef Level", "NONE");
  }

  public Command displayUpcomingReefLevel() {
    return Commands.none();
  }

  public int upcomingReefLevel() {
    return 0;
  }
}
