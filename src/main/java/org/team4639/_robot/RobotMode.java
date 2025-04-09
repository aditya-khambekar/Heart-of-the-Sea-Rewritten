package org.team4639._robot;

public class RobotMode {
  private static byte RobotMode = 0b0;
  // 0 = COMP
  // 1 = MANUAL

  public static boolean isComp() {
    return RobotMode == 0b0;
  }

  public static boolean isManual() {
    return RobotMode == 0b1;
  }

  public static void setRobotMode(byte robotMode) {
    RobotMode = robotMode;
  }
}
