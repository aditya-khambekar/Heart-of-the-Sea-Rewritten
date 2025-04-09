package org.team4639._robot;

public class RobotMode {
  private static byte RobotMode = 0b00;
  // 0 = COMP
  // 1 = SYSID
  // 2 = MANUAL

  public static boolean isComp() {
    return RobotMode == 0b00;
  }

  public static boolean isSysID() {
    return RobotMode == 0b01;
  }

  public static boolean isManual() {
    return RobotMode == 0b10;
  }

  public static void setRobotMode(byte robotMode) {
    RobotMode = robotMode;
  }
}
