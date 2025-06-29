package org.team4639.robot.commands;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import java.util.List;
import java.util.Set;
import org.team4639.robot.commands.superstructure.HomingCommand;
import org.team4639.robot.commands.superstructure.SuperstructureCommand;
import org.team4639.robot.constants.FieldConstants;
import org.team4639.robot.robot.Subsystems;
import org.team4639.robot.subsystems.superstructure.Superstructure;
import org.team4639.robot.subsystems.superstructure.SuperstructureSetpoints;

/** Placeholder class. All of these are commands.none right now while i get things sorted out. */
public class SuperstructureCommands {
  public static final Command IDLE = new SuperstructureCommand(SuperstructureSetpoints.IDLE);
  public static final Command HP = new SuperstructureCommand(SuperstructureSetpoints.HP);
  public static final Command HP_LOWER =
      new SuperstructureCommand(SuperstructureSetpoints.HP_LOWER);
  public static final Command CORAL_STOW =
      new SuperstructureCommand(SuperstructureSetpoints.CORAL_STOW);
  public static final Command ELEVATOR_READY =
      new SuperstructureCommand(SuperstructureSetpoints.ELEVATOR_READY);
  public static final Command L1 = new SuperstructureCommand(SuperstructureSetpoints.L1);
  public static final Command L2 = new SuperstructureCommand(SuperstructureSetpoints.L2);
  public static final Command L3 = new SuperstructureCommand(SuperstructureSetpoints.L3);
  public static final Command L4 = new SuperstructureCommand(SuperstructureSetpoints.L4);
  public static final Command L2_ALGAE =
      new SuperstructureCommand(SuperstructureSetpoints.L2_ALGAE);
  public static final Command L3_ALGAE =
      new SuperstructureCommand(SuperstructureSetpoints.L3_ALGAE);
  public static final Command BARGE = new SuperstructureCommand(SuperstructureSetpoints.BARGE);
  public static final Command ALGAE_STOW = Commands.none();
  public static final Command HOMING_READY =
      new SuperstructureCommand(SuperstructureSetpoints.HOMING_READY);
  public static final Command HOMING = new HomingCommand();

  public static final Command HOLD =
      Commands.defer(
          () -> new SuperstructureCommand(Superstructure.getSuperstructureState()),
          Set.of(
              Subsystems.superstructure, Subsystems.elevator, Subsystems.wrist, Subsystems.roller));

  public static final Command REJECT_CORAL =
      new SuperstructureCommand(SuperstructureSetpoints.REJECT_CORAL);
  public static final Command REJECT_ALGAE =
      new SuperstructureCommand(SuperstructureSetpoints.REJECT_ALGAE);

  public static final Command ALGAE_INTAKE =
      Commands.select(
          FieldConstants.ReefCenterPoseToAlgaeLocation,
          () -> {
            Pose2d drivePose = Subsystems.drive.getPose();

            return drivePose.nearest(
                List.of(
                    FieldConstants.TargetPositions.REEF_AB.getPose(),
                    FieldConstants.TargetPositions.REEF_CD.getPose(),
                    FieldConstants.TargetPositions.REEF_EF.getPose(),
                    FieldConstants.TargetPositions.REEF_GH.getPose(),
                    FieldConstants.TargetPositions.REEF_IJ.getPose(),
                    FieldConstants.TargetPositions.REEF_KL.getPose()));
          });
}
