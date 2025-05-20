package org.team4639.commands;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import org.team4639._robot.Subsystems;
import org.team4639.constants.FieldConstants;
import org.team4639.subsystems.elevator.ElevatorConstants;

public class SuperstructureCommands {
  public static Command intakeCoral() {
    var command =
        Commands.sequence(
            Subsystems.elevator
                .runToSetpoint(
                    ElevatorConstants.ProportionToPosition.convert(
                        ElevatorConstants.Setpoints.HP_PROPORTION))
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
            Subsystems.elevator.runToSetpoint(ElevatorConstants.Setpoints.SCORE_READY_PROPORTION)),
        Subsystems.elevator
            .runToSetpoint(scoringPositionSetpoint)
            .until(Subsystems.elevator::atPosition),
        Commands.deadline(
            Subsystems.scoring.intakeCoral(),
            Subsystems.elevator.runToSetpoint(scoringPositionSetpoint)));
  }

  // TODO: Uncomment scoring logic when elevator done
  public static Command score(double scoringPositionSetpoint, Pose2d destinationPose) {
    //    return Commands.sequence(
    //        Commands.deadline(
    //            Subsystems.drive.defer(
    //                () -> DriveCommands.alignHeadOn(Subsystems.drive, destinationPose)),
    //
    // Subsystems.elevator.runToSetpoint(ElevatorConstants.Setpoints.SCORE_READY_PROPORTION)),
    //        Subsystems.elevator
    //            .runToSetpoint(scoringPositionSetpoint)
    //            .until(Subsystems.elevator::atPosition),
    //        Commands.deadline(
    //            Subsystems.scoring.intakeCoral(),
    //            Subsystems.elevator.runToSetpoint(scoringPositionSetpoint)));
    return Subsystems.drive.defer(
        () ->
            DriveCommands.PIDtowithVelocityReset(
                Subsystems.drive, Subsystems.drive.getPose(), destinationPose));
  }

  public static Command intakeAlgae() {
    var drivetrainPose = Subsystems.drive.getPose();
    var nearestReefFacePose =
        drivetrainPose.nearest(
            FieldConstants.ReefCenterPoseToAlgaeLocation.keySet().stream().toList());
    double elevatorSetpointProportion =
        FieldConstants.ReefCenterPoseToAlgaeLocation.get(nearestReefFacePose) == 0b0
            ? ElevatorConstants.Setpoints.L2_ALGAE_PROPORTION
            : ElevatorConstants.Setpoints.L3_ALGAE_PROPORTION;

    return Commands.sequence(
        Commands.deadline(
            DriveCommands.reefAlign(Subsystems.drive),
            Subsystems.elevator.runToSetpoint(ElevatorConstants.Setpoints.SCORE_READY_PROPORTION)),
        Subsystems.elevator
            .runToSetpoint(
                ElevatorConstants.ProportionToPosition.convert(elevatorSetpointProportion))
            .until(Subsystems.elevator::atPosition),
        Commands.deadline(
            Subsystems.scoring.intakeAlgae(),
            Subsystems.elevator.runToSetpoint(
                ElevatorConstants.ProportionToPosition.convert(elevatorSetpointProportion)),
            DriveCommands.robotOrientedDrive(Subsystems.drive, new ChassisSpeeds(0.1, 0.0, 0.0))));
  }

  public static Command intakeAlgae(Pose2d targetPose) {
    //    var nearestReefFacePose =
    //        targetPose.nearest(
    //            FieldConstants.ReefCenterPoseToAlgaeLocation.keySet().stream().toList());
    //    double elevatorSetpointProportion =
    //        FieldConstants.ReefCenterPoseToAlgaeLocation.get(nearestReefFacePose) == 0b0
    //            ? ElevatorConstants.Setpoints.L2_ALGAE_PROPORTION
    //            : ElevatorConstants.Setpoints.L3_ALGAE_PROPORTION;
    //
    //            return Commands.sequence(
    //        Commands.deadline(
    //            DriveCommands.reefAlign(Subsystems.drive),
    //
    // Subsystems.elevator.runToSetpoint(ElevatorConstants.Setpoints.SCORE_READY_PROPORTION)),
    //        Subsystems.elevator
    //            .runToSetpoint(
    //                ElevatorConstants.ProportionToPosition.convert(elevatorSetpointProportion))
    //            .until(Subsystems.elevator::atPosition),
    //        Commands.deadline(
    //            Subsystems.scoring.intakeAlgae(),
    //            Subsystems.elevator.runToSetpoint(
    //                ElevatorConstants.ProportionToPosition.convert(elevatorSetpointProportion)),
    //            DriveCommands.robotOrientedDrive(Subsystems.drive, new ChassisSpeeds(0.1, 0.0,
    // 0.0))));

    return Subsystems.drive.defer(
        () ->
            DriveCommands.PIDtowithVelocityReset(
                Subsystems.drive, Subsystems.drive.getPose(), targetPose));
  }
}
