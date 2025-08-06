package org.team4639.robot.robot;

import org.team4639.lib.led.subsystem.PhysicalLEDStrip;
import org.team4639.robot.subsystems.DashboardOutputs;
import org.team4639.robot.subsystems.LimelightFlash;
import org.team4639.robot.subsystems.drive.Drive;
import org.team4639.robot.subsystems.superstructure.Superstructure;
import org.team4639.robot.subsystems.superstructure.elevator.ElevatorSubsystem;
import org.team4639.robot.subsystems.superstructure.roller.RollerSubsystem;
import org.team4639.robot.subsystems.superstructure.wrist.WristSubsystem;
import org.team4639.robot.subsystems.vision.Vision;

public class Subsystems {
  public static Drive drive;
  public static Vision vision;
  public static ElevatorSubsystem elevator;
  public static WristSubsystem wrist;
  public static RollerSubsystem roller;
  public static Superstructure superstructure;
  public static LimelightFlash limelightFlash;
  public static PhysicalLEDStrip leds;
}
