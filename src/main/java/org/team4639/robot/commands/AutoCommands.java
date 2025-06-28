package org.team4639.robot.commands;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.wpilibj2.command.Command;
import org.team4639.robot.robot.Subsystems;
import org.team4639.robot.subsystems.superstructure.Superstructure;
import org.team4639.robot.subsystems.superstructure.SuperstructureSetpoints;

public class AutoCommands {
  public static Command scoreL4(Pose2d pose) {
    return (Subsystems.reefTracker.setCurrentReefPoseCommand(pose)).andThen(((DriveCommands.PIDtoReefWithVelocityReset(
                    Subsystems.drive, Subsystems.drive.getPose(), pose)
                .alongWith(SuperstructureCommands.ELEVATOR_READY))
            .andThen(DriveCommands.stopWithX().alongWith(SuperstructureCommands.L4))
            .until(Subsystems.wrist::doesNotHaveCoral))
        .andThen(SuperstructureCommands.IDLE)
        .until(
            () ->
                Superstructure.atPosition(
                    Superstructure.getSuperstructureState(), SuperstructureSetpoints.IDLE)))
            .andThen(Subsystems.reefTracker.scoreL4());
  }

  public static Command intakeLeft() {
    return DriveCommands.coralStationAlignLeft(Subsystems.drive)
        .andThen(DriveCommands.stopWithX().alongWith(SuperstructureCommands.HP_LOWER))
        .until(Subsystems.wrist::hasCoral);
  }

  public static Command intakeRight() {
    return DriveCommands.coralStationAlignLeft(Subsystems.drive)
        .andThen(DriveCommands.stopWithX().alongWith(SuperstructureCommands.HP_LOWER))
        .until(Subsystems.wrist::hasCoral);
  }
}
