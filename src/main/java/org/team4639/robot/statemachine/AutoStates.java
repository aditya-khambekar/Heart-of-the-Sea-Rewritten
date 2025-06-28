package org.team4639.robot.statemachine;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.wpilibj2.command.button.RobotModeTriggers;
import org.team4639.lib.statebased.State;
import org.team4639.robot.commands.DriveCommands;
import org.team4639.robot.commands.SuperstructureCommands;
import org.team4639.robot.robot.Subsystems;

public class AutoStates {
  public static State PATH_NOCORAL() {
    return new State("PATH_NOCORAL")
        .whileTrue(SuperstructureCommands.IDLE)
        .onTrigger(RobotModeTriggers.teleop(), () -> States.IDLE);
  }

  public static State PATH_WITHCORAL() {
    return new State("PATH_WITHCORAL")
        .whileTrue(SuperstructureCommands.CORAL_STOW)
        .onTrigger(RobotModeTriggers.teleop(), () -> States.CORAL_STOW);
  }

  public static State ALIGN_TO_SCORE(Pose2d pose) {
    return new State("ALIGN_TO_SCORE")
        .whileTrue(
            DriveCommands.PIDtoReefWithVelocityReset(
                Subsystems.drive, Subsystems.drive.getPose(), pose),
            SuperstructureCommands.ELEVATOR_READY)
        .onTrigger(RobotModeTriggers.teleop(), () -> States.CORAL_STOW);
  }

  public static State SCOREL4() {
    return new State("SCORE")
        .whileTrue(SuperstructureCommands.L4)
        .onTrigger(RobotModeTriggers.teleop(), () -> States.L4_CORAL_SCORE);
  }

  public static State ALIGN_HP(Pose2d pose) {
    return new State("ALIGN_HP")
        .whileTrue(
            DriveCommands.PIDtowithVelocityReset(
                Subsystems.drive, Subsystems.drive.getPose(), pose),
            SuperstructureCommands.HP)
        .onTrigger(RobotModeTriggers.teleop(), () -> States.IDLE);
  }

  public static State HP() {
    return new State("HP")
        .whileTrue(SuperstructureCommands.HP)
        .onTrigger(RobotModeTriggers.teleop(), () -> States.HP_NODIR);
  }

  public static State NONE() {
    return new State("AUTO_NONE");
  }
}
