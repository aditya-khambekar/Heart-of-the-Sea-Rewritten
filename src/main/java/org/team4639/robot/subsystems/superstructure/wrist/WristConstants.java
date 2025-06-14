package org.team4639.robot.subsystems.superstructure.wrist;

import org.team4639.lib.tunable.TunableNumber;

public class WristConstants {
  public static TunableNumber wristKp =
      new TunableNumber().withDefaultValue(40.0).send("Scoring PIDs/Wrist kP");
  public static TunableNumber wristKi =
      new TunableNumber().withDefaultValue(0.0).send("Scoring PIDs/Wrist kI");
  public static TunableNumber wristKd =
      new TunableNumber().withDefaultValue(0).send("Scoring PIDs/Wrist kD");
  public static TunableNumber wristVelocity =
      new TunableNumber().withDefaultValue(60).send("Scoring PIDs/Wrist Velocity");
  public static TunableNumber wristAcceleration =
      new TunableNumber().withDefaultValue(20).send("Scoring PIDs/Wrist Acceleration");
}
