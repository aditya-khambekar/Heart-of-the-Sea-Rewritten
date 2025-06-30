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

public class SuperstructureCommands {
  public static Command IDLE;
  public static Command HP;
  public static Command HP_LOWER;
  public static Command CORAL_STOW;
  public static Command ELEVATOR_READY;
  public static Command L1;
  public static Command L2;
  public static Command L3;
  public static Command L4;
  public static Command L2_ALGAE;
  public static Command L3_ALGAE;
  public static Command BARGE;
  public static Command ALGAE_STOW;
  public static Command HOMING_READY;
  public static Command HOMING;
  public static Command HOLD;
  public static Command REJECT_CORAL;
  public static Command REJECT_ALGAE;
  public static Command ALGAE_INTAKE;

  public static void initCommands() {
    IDLE =
        Commands.defer(
                () -> new SuperstructureCommand(SuperstructureSetpoints.IDLE),
                Set.of(
                    Subsystems.elevator,
                    Subsystems.wrist,
                    Subsystems.roller,
                    Subsystems.superstructure))
            .withName("IDLE");
    HP =
        Commands.defer(
                () -> new SuperstructureCommand(SuperstructureSetpoints.HP).flashOnDone(),
                Set.of(
                    Subsystems.elevator,
                    Subsystems.wrist,
                    Subsystems.roller,
                    Subsystems.superstructure))
            .withName("HP");
    HP_LOWER =
        Commands.defer(
                () -> new SuperstructureCommand(SuperstructureSetpoints.HP_LOWER),
                Set.of(
                    Subsystems.elevator,
                    Subsystems.wrist,
                    Subsystems.roller,
                    Subsystems.superstructure))
            .withName("HP_LOWER");
    CORAL_STOW =
        Commands.defer(
                () -> new SuperstructureCommand(SuperstructureSetpoints.CORAL_STOW),
                Set.of(
                    Subsystems.elevator,
                    Subsystems.wrist,
                    Subsystems.roller,
                    Subsystems.superstructure))
            .withName("CORAL_STOW");
    ELEVATOR_READY =
        Commands.defer(
                () -> new SuperstructureCommand(SuperstructureSetpoints.ELEVATOR_READY),
                Set.of(
                    Subsystems.elevator,
                    Subsystems.wrist,
                    Subsystems.roller,
                    Subsystems.superstructure))
            .withName("ELEVATOR_READY");
    L1 =
        Commands.defer(
                () -> new SuperstructureCommand(SuperstructureSetpoints.L1).flashOnDone(),
                Set.of(
                    Subsystems.elevator,
                    Subsystems.wrist,
                    Subsystems.roller,
                    Subsystems.superstructure))
            .finallyDo(Subsystems.reefTracker::scoreL1Raw)
            .withName("L1");
    L2 =
        Commands.defer(
                () -> new SuperstructureCommand(SuperstructureSetpoints.L2).flashOnDone(),
                Set.of(
                    Subsystems.elevator,
                    Subsystems.wrist,
                    Subsystems.roller,
                    Subsystems.superstructure))
            .finallyDo(Subsystems.reefTracker::scoreL2Raw)
            .withName("L2");
    L3 =
        Commands.defer(
                () -> new SuperstructureCommand(SuperstructureSetpoints.L3).flashOnDone(),
                Set.of(
                    Subsystems.elevator,
                    Subsystems.wrist,
                    Subsystems.roller,
                    Subsystems.superstructure))
            .finallyDo(Subsystems.reefTracker::scoreL3Raw)
            .withName("L3");
    L4 =
        Commands.defer(
                () -> new SuperstructureCommand(SuperstructureSetpoints.L4).flashOnDone(),
                Set.of(
                    Subsystems.elevator,
                    Subsystems.wrist,
                    Subsystems.roller,
                    Subsystems.superstructure))
            .finallyDo(Subsystems.reefTracker::scoreL4Raw)
            .withName("L4");
    L2_ALGAE =
        Commands.defer(
                () -> new SuperstructureCommand(SuperstructureSetpoints.L2_ALGAE),
                Set.of(
                    Subsystems.elevator,
                    Subsystems.wrist,
                    Subsystems.roller,
                    Subsystems.superstructure))
            .withName("L2_ALGAE");
    L3_ALGAE =
        Commands.defer(
                () -> new SuperstructureCommand(SuperstructureSetpoints.L3_ALGAE),
                Set.of(
                    Subsystems.elevator,
                    Subsystems.wrist,
                    Subsystems.roller,
                    Subsystems.superstructure))
            .withName("L3_ALGAE");
    BARGE =
        Commands.defer(
                () -> new SuperstructureCommand(SuperstructureSetpoints.BARGE),
                Set.of(
                    Subsystems.elevator,
                    Subsystems.wrist,
                    Subsystems.roller,
                    Subsystems.superstructure))
            .withName("BARGE");
    ALGAE_STOW =
        Commands.defer(
                () -> Commands.none(),
                Set.of(
                    Subsystems.elevator,
                    Subsystems.wrist,
                    Subsystems.roller,
                    Subsystems.superstructure))
            .withName("ALGAE_STOW");
    HOMING_READY =
        Commands.defer(
                () -> new SuperstructureCommand(SuperstructureSetpoints.HOMING_READY),
                Set.of(
                    Subsystems.elevator,
                    Subsystems.wrist,
                    Subsystems.roller,
                    Subsystems.superstructure))
            .withName("HOMING_READY");
    HOMING =
        Commands.defer(
                () -> new HomingCommand(),
                Set.of(
                    Subsystems.elevator,
                    Subsystems.wrist,
                    Subsystems.roller,
                    Subsystems.superstructure))
            .withName("HOMING");

    HOLD =
        Commands.defer(
                () -> new SuperstructureCommand(Superstructure.getSuperstructureState()),
                Set.of(
                    Subsystems.superstructure,
                    Subsystems.elevator,
                    Subsystems.wrist,
                    Subsystems.roller))
            .withName("HOLD");

    REJECT_CORAL =
        Commands.defer(
                () -> new SuperstructureCommand(SuperstructureSetpoints.REJECT_CORAL),
                Set.of(
                    Subsystems.elevator,
                    Subsystems.wrist,
                    Subsystems.roller,
                    Subsystems.superstructure))
            .withName("REJECT_CORAL");
    REJECT_ALGAE =
        Commands.defer(
                () -> new SuperstructureCommand(SuperstructureSetpoints.REJECT_ALGAE),
                Set.of(
                    Subsystems.elevator,
                    Subsystems.wrist,
                    Subsystems.roller,
                    Subsystems.superstructure))
            .withName("REJECT_ALGAE");

    ALGAE_INTAKE =
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
                })
            .withName("ALGAE_INTAKE");
  }

  // Method versions that create new commands each time
  public static Command idle() {
    return new SuperstructureCommand(SuperstructureSetpoints.IDLE);
  }

  public static Command hp() {
    return new SuperstructureCommand(SuperstructureSetpoints.HP).flashOnDone();
  }

  public static Command hpLower() {
    return new SuperstructureCommand(SuperstructureSetpoints.HP_LOWER).flashOnDone();
  }

  public static Command coralStow() {
    return new SuperstructureCommand(SuperstructureSetpoints.CORAL_STOW);
  }

  public static Command elevatorReady() {
    return new SuperstructureCommand(SuperstructureSetpoints.ELEVATOR_READY);
  }

  public static Command l1() {
    return new SuperstructureCommand(SuperstructureSetpoints.L1).flashOnDone();
  }

  public static Command l2() {
    return new SuperstructureCommand(SuperstructureSetpoints.L2).flashOnDone();
  }

  public static Command l3() {
    return new SuperstructureCommand(SuperstructureSetpoints.L3).flashOnDone();
  }

  public static Command l4() {
    return new SuperstructureCommand(SuperstructureSetpoints.L4).flashOnDone();
  }

  public static Command l2Algae() {
    return new SuperstructureCommand(SuperstructureSetpoints.L2_ALGAE);
  }

  public static Command l3Algae() {
    return new SuperstructureCommand(SuperstructureSetpoints.L3_ALGAE);
  }

  public static Command barge() {
    return new SuperstructureCommand(SuperstructureSetpoints.BARGE);
  }

  public static Command algaeStow() {
    return Commands.none();
  }

  public static Command homingReady() {
    return new SuperstructureCommand(SuperstructureSetpoints.HOMING_READY);
  }

  public static Command homing() {
    return new HomingCommand();
  }

  public static Command hold() {
    return Commands.defer(
        () -> new SuperstructureCommand(Superstructure.getSuperstructureState()),
        Set.of(
            Subsystems.superstructure, Subsystems.elevator, Subsystems.wrist, Subsystems.roller));
  }

  public static Command rejectCoral() {
    return new SuperstructureCommand(SuperstructureSetpoints.REJECT_CORAL);
  }

  public static Command rejectAlgae() {
    return new SuperstructureCommand(SuperstructureSetpoints.REJECT_ALGAE);
  }

  public static Command algaeIntake() {
    return Commands.select(
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
}
