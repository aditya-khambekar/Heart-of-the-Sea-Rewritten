package org.team4639.commands;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import org.team4639._robot.Subsystems;
import org.team4639.subsystems.elevator.ElevatorConstants;

public class SuperstructureCommands {
  public static Command intakeCoral() {
    var command =
        Commands.sequence(
            Subsystems.elevator
                .runToSetpoint(
                    ElevatorConstants.ProportionToPosition.convert(
                        ElevatorConstants.Setpoints.HP_Proportion))
                .until(Subsystems.elevator::atPosition),
            Subsystems.scoring.intakeCoral());
    return command;
  }

  /**
   * Automates scoring a coral in the reef.
   *
   * @param scoringPositionSetpoint the elevator setpoint that corresponds to the scoring position.
   * @param direction 0 for left, 1 for right
   * @return A command which auto-aligns and scores coral.
   */
  public static Command score(double scoringPositionSetpoint, int direction) {
    return Commands.sequence(
        Commands.deadline(
            direction == 0
                ? DriveCommands.reefAlignLeft(Subsystems.drive)
                : DriveCommands.reefAlignRight(Subsystems.drive),
            Subsystems.elevator.runToSetpoint(ElevatorConstants.Setpoints.SCORE_READY_POSITION)),
        Subsystems.elevator
            .runToSetpoint(scoringPositionSetpoint)
            .until(Subsystems.elevator::atPosition),
        Commands.deadline(
            Subsystems.scoring.intakeCoral(),
            Subsystems.elevator.runToSetpoint(scoringPositionSetpoint)));
  }
}
