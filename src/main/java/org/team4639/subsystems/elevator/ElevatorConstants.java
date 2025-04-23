package org.team4639.subsystems.elevator;

import org.team4639._lib.UnitConvertor;
import org.team4639._lib.annotation.Untuned;

public class ElevatorConstants {
  static class Params {
    static double kp = 1.798;
    static double ki = 0.0;
    static double kd = 0.0;

    static double ks = 0.27919;
    static double kg = 2.0;
    static double kv = 0.11358;
    static double ka = 0.0;

    static double acceleration = 190.0;
    static double velocity = 300.0;
  }

  public static class Setpoints {
    @Untuned public static final double IDLE_Proportion = 0.04;
    public static final double HP_Proportion = 0.048;
    public static final double Processor_Proportion = 0.0513;
    public static final double L1_Proportion = 0.15;

    public static final double L2_Proportion = 0.2893; // 0.28
    public static final double L3_Proportion = 0.496; // 0.481
    public static final double L4_Proportion = 0.85635;

    public static final double L2_ALGAE_Proportion = 0.32;
    public static final double L3_ALGAE_Proportion = 0.5054;
    public static final double Barge_Proportion = .95;

    public static final double Ground_Intake_Proportion = 0.0;
    public static final double Homing_Proportion = 0.0;
    public static final double ELEVATOR_LOWEST_PROPORTION = 0.0;

    public static final double SCORE_READY_POSITION = 0.35;
  }

  public static double UP_POSITION = 64;
  public static double DOWN_POSITION = -0.7;

  public static double ELEVATOR_TOLERANCE = 0.5;

  public static final int statorCurrentLimit = 45;
  public static final int supplyCurrentLimit = 50;

  public static final UnitConvertor<Double, Double> ProportionToPosition =
      UnitConvertor.linearConvertingRange(0, 1, DOWN_POSITION, UP_POSITION);
}
