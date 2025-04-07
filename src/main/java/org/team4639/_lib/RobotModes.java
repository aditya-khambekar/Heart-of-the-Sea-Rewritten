package org.team4639._lib;

public class RobotModes {
  private static boolean tuningMode = false;

  public static boolean isTuningMode() {
    return tuningMode;
  }

  public static void setTuningMode(boolean tuningMode) {
    RobotModes.tuningMode = tuningMode;
  }
}
