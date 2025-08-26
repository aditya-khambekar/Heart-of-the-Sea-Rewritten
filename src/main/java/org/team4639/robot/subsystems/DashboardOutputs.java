package org.team4639.robot.subsystems;

import edu.wpi.first.wpilibj.Notifier;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import java.util.Objects;
import lombok.Getter;
import lombok.Setter;

public class DashboardOutputs {

  private static volatile DashboardOutputs instance;
  @Getter @Setter private int upcomingReefLevel = 0;

  @Getter private final Trigger selectedL4 = new Trigger(() -> upcomingReefLevel == 4);
  @Getter private final Trigger selectedL3 = new Trigger(() -> upcomingReefLevel == 3);
  @Getter private final Trigger selectedL2 = new Trigger(() -> upcomingReefLevel == 2);
  @Getter private final Trigger selectedL1 = new Trigger(() -> upcomingReefLevel == 1);

  public static synchronized DashboardOutputs getInstance() {
    return instance = Objects.requireNonNullElseGet(instance, DashboardOutputs::new);
  }

  private DashboardOutputs() {
    SmartDashboard.putString("Reef Level", "NONE");

    new Notifier(() -> SmartDashboard.putNumber("DOUT", getUpcomingReefLevel()))
        .startPeriodic(upcomingReefLevel);
  }

  public Command displayUpcomingReefLevel() {
    return Commands.run(
        () -> SmartDashboard.putString("Reef Level", String.valueOf(upcomingReefLevel)));
  }

  public Command selectL4() {
    return Commands.runOnce(() -> setUpcomingReefLevel(4));
  }

  public Command selectL3() {
    return Commands.runOnce(() -> setUpcomingReefLevel(3));
  }

  public Command selectL2() {
    return Commands.runOnce(() -> setUpcomingReefLevel(2));
  }

  public Command selectL1() {
    return Commands.runOnce(() -> setUpcomingReefLevel(1));
  }
}
