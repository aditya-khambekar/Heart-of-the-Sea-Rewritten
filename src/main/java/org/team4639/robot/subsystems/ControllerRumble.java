package org.team4639.robot.subsystems;

import static edu.wpi.first.units.Units.Seconds;

import edu.wpi.first.units.measure.MutTime;
import edu.wpi.first.wpilibj.GenericHID;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import org.team4639.robot.robot.RobotContainer;

public final class ControllerRumble extends SubsystemBase {
  public static final double RUMBLE_MAGNITUDE = 0.5;
  public static final double RUMBLE_TIME_SEC = 0.5;
  private MutTime timeSinceLastCall = Seconds.mutable(1000);

  public ControllerRumble() {}

  public void rumble() {
    timeSinceLastCall.mut_replace(0, Seconds);
  }

  @Override
  public void periodic() {
    timeSinceLastCall.mut_plus(0.02, Seconds);

    if (timeSinceLastCall.lte(Seconds.of(RUMBLE_TIME_SEC))) {
      RobotContainer.driver.setRumble(GenericHID.RumbleType.kBothRumble, RUMBLE_MAGNITUDE);
      SmartDashboard.putBoolean("Controller Rumble", true);
    } else {
      RobotContainer.driver.setRumble(GenericHID.RumbleType.kBothRumble, 0.0);
      SmartDashboard.putBoolean("Controller Rumble", false);
    }
  }

  public Command rumbleOnDone() {
    return Commands.idle().finallyDo(this::rumble);
  }
}
