package org.team4639.robot.statemachine;

import org.team4639.robot.commands.DriveCommands;
import org.team4639.robot.commands.SuperstructureCommands;
import org.team4639.robot.modaltriggers.DriveTriggers;
import org.team4639.robot.robot.Subsystems;

public class States {
  public static final State IDLE = new State("IDLE").whileTrue(SuperstructureCommands.IDLE);

  public static final State INTAKE_LEFT =
      new State("INTAKE_LEFT")
          .whileTrue(
              DriveCommands.coralStationAlignLeft(Subsystems.drive), SuperstructureCommands.INTAKE)
          .withEndCondition(DriveTriggers.closeToLeftStation.negate(), States.INTAKE_II)
          .onEmergency(IDLE);

  public static final State INTAKE_RIGHT =
      new State("INTAKE_RIGHT")
          .whileTrue(
              DriveCommands.coralStationAlignRight(Subsystems.drive), SuperstructureCommands.INTAKE)
          .withEndCondition(DriveTriggers.closeToRightStation.negate(), States.INTAKE_II)
          .onEmergency(IDLE);

  public static final State INTAKE_II =
      new State("INTAKE_II")
          .whileTrue(SuperstructureCommands.INTAKE_II)
          .withEndCondition(Subsystems.wrist::hasCoral, States.CORAL_STOW)
          .onEmergency(IDLE);

  public static final State CORAL_STOW =
      new State("CORAL_STOW")
          .whileTrue(SuperstructureCommands.CORAL_STOW)
          .whileTrue(SuperstructureCommands.CORAL_STOW);

  public static final State CORAL_SCORE_ALIGN =
      new State("CORAL_SCORE_ALIGN")
          .whileTrue(
              SuperstructureCommands.ELEVATOR_READY, DriveCommands.reefAlign(Subsystems.drive))
          .withEndCondition(
              () -> Subsystems.drive.getCurrentCommand().getName().endsWith("Joystick"),
              States.CHOOSE_CORAL_LEVEL)
          .onEmergency(CORAL_STOW)
          .onAccelerationLimit(CORAL_STOW);

  public static final State CHOOSE_CORAL_LEVEL =
      new State("CHOOSE_CORAL_LEVEL")
          .whileTrue(
              SuperstructureCommands.HOLD,
              Subsystems.drive.run(() -> Subsystems.drive.stopWithX()));
  // TODO: Coral Scoring Levels as end conditions for this
}
