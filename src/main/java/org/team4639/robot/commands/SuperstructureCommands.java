package org.team4639.robot.commands;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import java.util.List;
import java.util.Set;
import org.team4639.robot.commands.superstructure.HomingCommand;
import org.team4639.robot.commands.superstructure.MotionSuperstructureCommand;
import org.team4639.robot.constants.reefscape.FieldConstants;
import org.team4639.robot.constants.reefscape.TargetPositions;
import org.team4639.robot.robot.Subsystems;
import org.team4639.robot.subsystems.superstructure.Superstructure;
import org.team4639.robot.subsystems.superstructure.SuperstructureSetpoints;

public final class SuperstructureCommands {
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
            () -> new MotionSuperstructureCommand(SuperstructureSetpoints.IDLE, "IDLE"),
            Set.of(
                Subsystems.elevator,
                Subsystems.wrist,
                Subsystems.roller,
                Subsystems.superstructure));
    HP =
        Commands.defer(
            () -> new MotionSuperstructureCommand(SuperstructureSetpoints.HP, "HP").flashOnDone(),
            Set.of(
                Subsystems.elevator,
                Subsystems.wrist,
                Subsystems.roller,
                Subsystems.superstructure));
    HP_LOWER =
        Commands.defer(
            () -> new MotionSuperstructureCommand(SuperstructureSetpoints.HP_LOWER, "HP_LOWER"),
            Set.of(
                Subsystems.elevator,
                Subsystems.wrist,
                Subsystems.roller,
                Subsystems.superstructure));
    CORAL_STOW =
        Commands.defer(
            () -> new MotionSuperstructureCommand(SuperstructureSetpoints.CORAL_STOW, "CORAL_STOW"),
            Set.of(
                Subsystems.elevator,
                Subsystems.wrist,
                Subsystems.roller,
                Subsystems.superstructure));
    ELEVATOR_READY =
        Commands.defer(
            () ->
                new MotionSuperstructureCommand(
                    SuperstructureSetpoints.ELEVATOR_READY, "ELEVATOR_READY"),
            Set.of(
                Subsystems.elevator,
                Subsystems.wrist,
                Subsystems.roller,
                Subsystems.superstructure));
    L1 =
        Commands.defer(
            () -> new MotionSuperstructureCommand(SuperstructureSetpoints.L1, "L1").flashOnDone(),
            Set.of(
                Subsystems.elevator,
                Subsystems.wrist,
                Subsystems.roller,
                Subsystems.superstructure));
    L2 =
        Commands.defer(
            () -> new MotionSuperstructureCommand(SuperstructureSetpoints.L2, "L2").flashOnDone(),
            Set.of(
                Subsystems.elevator,
                Subsystems.wrist,
                Subsystems.roller,
                Subsystems.superstructure));
    L3 =
        Commands.defer(
            () -> new MotionSuperstructureCommand(SuperstructureSetpoints.L3, "L3").flashOnDone(),
            Set.of(
                Subsystems.elevator,
                Subsystems.wrist,
                Subsystems.roller,
                Subsystems.superstructure));
    L4 =
        Commands.defer(
            () -> new MotionSuperstructureCommand(SuperstructureSetpoints.L4, "L4").flashOnDone(),
            Set.of(
                Subsystems.elevator,
                Subsystems.wrist,
                Subsystems.roller,
                Subsystems.superstructure));
    L2_ALGAE =
        Commands.defer(
            () -> new MotionSuperstructureCommand(SuperstructureSetpoints.L2_ALGAE, "L2_ALGAE"),
            Set.of(
                Subsystems.elevator,
                Subsystems.wrist,
                Subsystems.roller,
                Subsystems.superstructure));
    L3_ALGAE =
        Commands.defer(
            () -> new MotionSuperstructureCommand(SuperstructureSetpoints.L3_ALGAE, "L3_ALGAE"),
            Set.of(
                Subsystems.elevator,
                Subsystems.wrist,
                Subsystems.roller,
                Subsystems.superstructure));
    BARGE =
        Commands.defer(
            () -> new MotionSuperstructureCommand(SuperstructureSetpoints.BARGE, "BARGE"),
            Set.of(
                Subsystems.elevator,
                Subsystems.wrist,
                Subsystems.roller,
                Subsystems.superstructure));
    ALGAE_STOW =
        Commands.defer(
            () -> new MotionSuperstructureCommand(SuperstructureSetpoints.ALGAE_STOW, "ALGAE_STOW"),
            Set.of(
                Subsystems.elevator,
                Subsystems.wrist,
                Subsystems.roller,
                Subsystems.superstructure));
    HOMING_READY =
        Commands.defer(
            () ->
                new MotionSuperstructureCommand(
                    SuperstructureSetpoints.HOMING_READY, "HOMING_READY"),
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
            () -> new MotionSuperstructureCommand(Superstructure.getSuperstructureState(), "HOLD"),
            Set.of(
                Subsystems.superstructure,
                Subsystems.elevator,
                Subsystems.wrist,
                Subsystems.roller));

    REJECT_CORAL =
        Commands.defer(
            () ->
                new MotionSuperstructureCommand(
                    SuperstructureSetpoints.REJECT_CORAL, "REJECT_CORAL"),
            Set.of(
                Subsystems.elevator,
                Subsystems.wrist,
                Subsystems.roller,
                Subsystems.superstructure));
    REJECT_ALGAE =
        Commands.defer(
            () ->
                new MotionSuperstructureCommand(
                    SuperstructureSetpoints.REJECT_ALGAE, "REJECT_ALGAE"),
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
                          TargetPositions.REEF_AB.getPose(),
                          TargetPositions.REEF_CD.getPose(),
                          TargetPositions.REEF_EF.getPose(),
                          TargetPositions.REEF_GH.getPose(),
                          TargetPositions.REEF_IJ.getPose(),
                          TargetPositions.REEF_KL.getPose()));
                })
            .withName("ALGAE_INTAKE");
  }

  // Method versions that create new commands each time
  public static MotionSuperstructureCommand idle() {
    return new MotionSuperstructureCommand(SuperstructureSetpoints.IDLE, "IDLE");
  }

  public static MotionSuperstructureCommand hp() {
    return new MotionSuperstructureCommand(SuperstructureSetpoints.HP, "HP").flashOnDone();
  }

  public static MotionSuperstructureCommand hpLower() {
    return new MotionSuperstructureCommand(SuperstructureSetpoints.HP_LOWER, "HP_LOWER")
        .flashOnDone();
  }

  public static MotionSuperstructureCommand coralStow() {
    return new MotionSuperstructureCommand(SuperstructureSetpoints.CORAL_STOW, "CORAL_STOW");
  }

  public static MotionSuperstructureCommand elevatorReady() {
    return new MotionSuperstructureCommand(
        SuperstructureSetpoints.ELEVATOR_READY, "ELEVATOR_READY");
  }

  public static MotionSuperstructureCommand l1() {
    return new MotionSuperstructureCommand(SuperstructureSetpoints.L1, "L1").flashOnDone();
  }

  public static MotionSuperstructureCommand l2() {
    return new MotionSuperstructureCommand(SuperstructureSetpoints.L2, "L2").flashOnDone();
  }

  public static MotionSuperstructureCommand l3() {
    return new MotionSuperstructureCommand(SuperstructureSetpoints.L3, "L3").flashOnDone();
  }

  public static MotionSuperstructureCommand l4() {
    return new MotionSuperstructureCommand(SuperstructureSetpoints.L4, "L4").flashOnDone();
  }

  public static MotionSuperstructureCommand l2Algae() {
    return new MotionSuperstructureCommand(SuperstructureSetpoints.L2_ALGAE, "L2_ALGAE");
  }

  public static MotionSuperstructureCommand l3Algae() {
    return new MotionSuperstructureCommand(SuperstructureSetpoints.L3_ALGAE, "L3_ALGAE");
  }

  public static MotionSuperstructureCommand barge() {
    return new MotionSuperstructureCommand(SuperstructureSetpoints.BARGE, "BARGE");
  }

  public static MotionSuperstructureCommand algaeStow() {
    return new MotionSuperstructureCommand(SuperstructureSetpoints.ALGAE_STOW, "ALGAE_STOW");
  }

  public static MotionSuperstructureCommand homingReady() {
    return new MotionSuperstructureCommand(SuperstructureSetpoints.HOMING_READY, "HOMING_READY");
  }

  public static Command homing() {
    return new HomingCommand();
  }

  public static Command hold() {
    return Commands.defer(
        () -> new MotionSuperstructureCommand(Superstructure.getSuperstructureState(), "HOLD"),
        Set.of(
            Subsystems.superstructure, Subsystems.elevator, Subsystems.wrist, Subsystems.roller));
  }

  public static MotionSuperstructureCommand rejectCoral() {
    return new MotionSuperstructureCommand(SuperstructureSetpoints.REJECT_CORAL, "REJECT_CORAL");
  }

  public static MotionSuperstructureCommand rejectAlgae() {
    return new MotionSuperstructureCommand(SuperstructureSetpoints.REJECT_ALGAE, "REJECT_ALGAE");
  }

  public static Command algaeIntake() {
    return Commands.select(
        FieldConstants.ReefCenterPoseToAlgaeLocation(),
        () -> {
          Pose2d drivePose = Subsystems.drive.getPose();

          return drivePose.nearest(
              List.of(
                  TargetPositions.REEF_AB.getPose(),
                  TargetPositions.REEF_CD.getPose(),
                  TargetPositions.REEF_EF.getPose(),
                  TargetPositions.REEF_GH.getPose(),
                  TargetPositions.REEF_IJ.getPose(),
                  TargetPositions.REEF_KL.getPose()));
        });
  }

  public static Command algaeIntake(Pose2d pose) {
    return Commands.select(
        FieldConstants.ReefCenterPoseToAlgaeLocation(),
        () -> {
          return pose.nearest(
              List.of(
                  TargetPositions.REEF_AB.getPose(),
                  TargetPositions.REEF_CD.getPose(),
                  TargetPositions.REEF_EF.getPose(),
                  TargetPositions.REEF_GH.getPose(),
                  TargetPositions.REEF_IJ.getPose(),
                  TargetPositions.REEF_KL.getPose()));
        });
  }
}
