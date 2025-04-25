package org.team4639.auto;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import org.team4639.commands.SuperstructureCommands;
import org.team4639.subsystems.elevator.ElevatorConstants;

public class AutoFactory {

  public static Command compileAuton() {

    return Commands.none();
  }

  public static record CoralScoringSegment(char location, int level) {}

  public static record AlgaeScoringSegment(Pose2d scoringPose) {}

  public static record CoralIntakeSegment(char intakeStationLocation) {}

  public static record AlgaeIntakeSegment(char[] location) {}

  // TODO: PID to pose instead of closest
  public static Command compileCoralScoringSegment(CoralScoringSegment segment) {
    var elevatorSetpoint =
        switch (segment.level) {
          case 1 -> ElevatorConstants.Setpoints.L1_PROPORTION;
          case 2 -> ElevatorConstants.Setpoints.L2_PROPORTION;
          case 3 -> ElevatorConstants.Setpoints.L3_PROPORTION;
          case 4 -> ElevatorConstants.Setpoints.L4_PROPORTION;

          default -> ElevatorConstants.Setpoints.SCORE_READY_PROPORTION;
        };

    var direction =
        switch (segment.location) {
          case 'A' -> 0b0;
          case 'B' -> 0b1;
          case 'C' -> 0b0;
          case 'D' -> 0b1;
          case 'E' -> 0b0;
          case 'F' -> 0b1;
          case 'G' -> 0b0;
          case 'H' -> 0b1;
          case 'I' -> 0b0;
          case 'J' -> 0b1;
          case 'K' -> 0b0;
          case 'L' -> 0b1;

          default -> 0b0;
        };

    return SuperstructureCommands.score(elevatorSetpoint, direction);
  }

  // TODO: pathfind to pose before doing this, probably unnecessary but
  public static Command compileCoralIntakeSegment(CoralIntakeSegment segment) {
    return SuperstructureCommands.intakeCoral();
  }

  // TODO: write pathfind to pose for this as well
  public static Command compileAlgaeIntakeSegment(AlgaeIntakeSegment segment) {
    return SuperstructureCommands.intakeAlgae();
  }
}
