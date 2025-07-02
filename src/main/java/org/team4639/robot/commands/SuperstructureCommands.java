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
            () -> new SuperstructureCommand(SuperstructureSetpoints.IDLE, "IDLE"),
            Set.of(
                Subsystems.elevator,
                Subsystems.wrist,
                Subsystems.roller,
                Subsystems.superstructure));
    HP =
        Commands.defer(
            () -> new SuperstructureCommand(SuperstructureSetpoints.HP, "HP").flashOnDone(),
            Set.of(
                Subsystems.elevator,
                Subsystems.wrist,
                Subsystems.roller,
                Subsystems.superstructure));
    HP_LOWER =
        Commands.defer(
            () -> new SuperstructureCommand(SuperstructureSetpoints.HP_LOWER, "HP_LOWER"),
            Set.of(
                Subsystems.elevator,
                Subsystems.wrist,
                Subsystems.roller,
                Subsystems.superstructure));
    CORAL_STOW =
        Commands.defer(
            () -> new SuperstructureCommand(SuperstructureSetpoints.CORAL_STOW, "CORAL_STOW"),
            Set.of(
                Subsystems.elevator,
                Subsystems.wrist,
                Subsystems.roller,
                Subsystems.superstructure));
    ELEVATOR_READY =
        Commands.defer(
            () ->
                new SuperstructureCommand(SuperstructureSetpoints.ELEVATOR_READY, "ELEVATOR_READY"),
            Set.of(
                Subsystems.elevator,
                Subsystems.wrist,
                Subsystems.roller,
                Subsystems.superstructure));
    L1 =
        Commands.defer(
                () -> new SuperstructureCommand(SuperstructureSetpoints.L1, "L1").flashOnDone(),
                Set.of(
                    Subsystems.elevator,
                    Subsystems.wrist,
                    Subsystems.roller,
                    Subsystems.superstructure))
            .finallyDo(Subsystems.reefTracker::scoreL1Raw);
    L2 =
        Commands.defer(
                () -> new SuperstructureCommand(SuperstructureSetpoints.L2, "L2").flashOnDone(),
                Set.of(
                    Subsystems.elevator,
                    Subsystems.wrist,
                    Subsystems.roller,
                    Subsystems.superstructure))
            .finallyDo(Subsystems.reefTracker::scoreL2Raw);
    L3 =
        Commands.defer(
                () -> new SuperstructureCommand(SuperstructureSetpoints.L3, "L3").flashOnDone(),
                Set.of(
                    Subsystems.elevator,
                    Subsystems.wrist,
                    Subsystems.roller,
                    Subsystems.superstructure))
            .finallyDo(Subsystems.reefTracker::scoreL3Raw);
    L4 =
        Commands.defer(
                () -> new SuperstructureCommand(SuperstructureSetpoints.L4, "L4").flashOnDone(),
                Set.of(
                    Subsystems.elevator,
                    Subsystems.wrist,
                    Subsystems.roller,
                    Subsystems.superstructure))
            .finallyDo(Subsystems.reefTracker::scoreL4Raw);
    L2_ALGAE =
        Commands.defer(
            () -> new SuperstructureCommand(SuperstructureSetpoints.L2_ALGAE, "L2_ALGAE"),
            Set.of(
                Subsystems.elevator,
                Subsystems.wrist,
                Subsystems.roller,
                Subsystems.superstructure));
    L3_ALGAE =
        Commands.defer(
            () -> new SuperstructureCommand(SuperstructureSetpoints.L3_ALGAE, "L3_ALGAE"),
            Set.of(
                Subsystems.elevator,
                Subsystems.wrist,
                Subsystems.roller,
                Subsystems.superstructure));
    BARGE =
        Commands.defer(
            () -> new SuperstructureCommand(SuperstructureSetpoints.BARGE, "BARGE"),
            Set.of(
                Subsystems.elevator,
                Subsystems.wrist,
                Subsystems.roller,
                Subsystems.superstructure));
    ALGAE_STOW =
        Commands.defer(
            () -> new SuperstructureCommand(SuperstructureSetpoints.ALGAE_STOW, "ALGAE_STOW"),
            Set.of(
                Subsystems.elevator,
                Subsystems.wrist,
                Subsystems.roller,
                Subsystems.superstructure));
    HOMING_READY =
        Commands.defer(
            () -> new SuperstructureCommand(SuperstructureSetpoints.HOMING_READY, "HOMING_READY"),
            Set.of(
                Subsystems.elevator,
                Subsystems.wrist,
                Subsystems.roller,
                Subsystems.superstructure));
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
            () -> new SuperstructureCommand(Superstructure.getSuperstructureState(), "HOLD"),
            Set.of(
                Subsystems.superstructure,
                Subsystems.elevator,
                Subsystems.wrist,
                Subsystems.roller));

    REJECT_CORAL =
        Commands.defer(
            () -> new SuperstructureCommand(SuperstructureSetpoints.REJECT_CORAL, "REJECT_CORAL"),
            Set.of(
                Subsystems.elevator,
                Subsystems.wrist,
                Subsystems.roller,
                Subsystems.superstructure));
    REJECT_ALGAE =
        Commands.defer(
            () -> new SuperstructureCommand(SuperstructureSetpoints.REJECT_ALGAE, "REJECT_ALGAE"),
            Set.of(
                Subsystems.elevator,
                Subsystems.wrist,
                Subsystems.roller,
                Subsystems.superstructure));

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
  public static SuperstructureCommand idle() {
    return new SuperstructureCommand(SuperstructureSetpoints.IDLE, "IDLE");
  }

  public static SuperstructureCommand hp() {
    return new SuperstructureCommand(SuperstructureSetpoints.HP, "HP").flashOnDone();
  }

  public static SuperstructureCommand hpLower() {
    return new SuperstructureCommand(SuperstructureSetpoints.HP_LOWER, "HP_LOWER").flashOnDone();
  }

  public static SuperstructureCommand coralStow() {
    return new SuperstructureCommand(SuperstructureSetpoints.CORAL_STOW, "CORAL_STOW");
  }

  public static SuperstructureCommand elevatorReady() {
    return new SuperstructureCommand(SuperstructureSetpoints.ELEVATOR_READY, "ELEVATOR_READY");
  }

  public static SuperstructureCommand l1() {
    return new SuperstructureCommand(SuperstructureSetpoints.L1, "L1").flashOnDone();
  }

  public static SuperstructureCommand l2() {
    return new SuperstructureCommand(SuperstructureSetpoints.L2, "L2").flashOnDone();
  }

  public static SuperstructureCommand l3() {
    return new SuperstructureCommand(SuperstructureSetpoints.L3, "L3").flashOnDone();
  }

  public static SuperstructureCommand l4() {
    return new SuperstructureCommand(SuperstructureSetpoints.L4, "L4").flashOnDone();
  }

  public static SuperstructureCommand l2Algae() {
    return new SuperstructureCommand(SuperstructureSetpoints.L2_ALGAE, "L2_ALGAE");
  }

  public static SuperstructureCommand l3Algae() {
    return new SuperstructureCommand(SuperstructureSetpoints.L3_ALGAE, "L3_ALGAE");
  }

  public static SuperstructureCommand barge() {
    return new SuperstructureCommand(SuperstructureSetpoints.BARGE, "BARGE");
  }

  public static SuperstructureCommand algaeStow() {
    return new SuperstructureCommand(SuperstructureSetpoints.ALGAE_STOW, "ALGAE_STOW");
  }

  public static SuperstructureCommand homingReady() {
    return new SuperstructureCommand(SuperstructureSetpoints.HOMING_READY, "HOMING_READY");
  }

  public static Command homing() {
    return new HomingCommand();
  }

  public static Command hold() {
    return Commands.defer(
        () -> new SuperstructureCommand(Superstructure.getSuperstructureState(), "HOLD"),
        Set.of(
            Subsystems.superstructure, Subsystems.elevator, Subsystems.wrist, Subsystems.roller));
  }

  public static SuperstructureCommand rejectCoral() {
    return new SuperstructureCommand(SuperstructureSetpoints.REJECT_CORAL, "REJECT_CORAL");
  }

  public static SuperstructureCommand rejectAlgae() {
    return new SuperstructureCommand(SuperstructureSetpoints.REJECT_ALGAE, "REJECT_ALGAE");
  }

  public static Command algaeIntake() {
    return Commands.select(
        FieldConstants.ReefCenterPoseToAlgaeLocation(),
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

  public static Command algaeIntake(Pose2d pose) {
    return Commands.select(
        FieldConstants.ReefCenterPoseToAlgaeLocation(),
        () -> {
          return pose.nearest(
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
