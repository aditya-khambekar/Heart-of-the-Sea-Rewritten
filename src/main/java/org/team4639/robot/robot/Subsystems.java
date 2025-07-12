package org.team4639.robot.robot;

import org.team4639.lib.led.subsystem.PhysicalLEDStrip;
import org.team4639.robot.subsystems.ControllerRumble;
import org.team4639.robot.subsystems.DashboardOutputs;
import org.team4639.robot.subsystems.LimelightFlash;
import org.team4639.robot.subsystems.drive.Drive;
import org.team4639.robot.subsystems.superstructure.Superstructure;
import org.team4639.robot.subsystems.superstructure.elevator.Elevator;
import org.team4639.robot.subsystems.superstructure.roller.Roller;
import org.team4639.robot.subsystems.superstructure.wrist.Wrist;
import org.team4639.robot.subsystems.vision.Vision;

public final class Subsystems {
  public static Drive drive;
  public static Vision vision;
  public static DashboardOutputs dashboardOutputs;
  public static Elevator elevator;
  public static Wrist wrist;
  public static Roller roller;
  public static Superstructure superstructure;
  public static LimelightFlash limelightFlash;
  public static PhysicalLEDStrip leds;
  public static ControllerRumble controllerRumble;
}
