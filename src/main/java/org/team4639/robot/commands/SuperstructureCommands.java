package org.team4639.robot.commands;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import java.util.List;
import java.util.Set;
import org.team4639.robot.commands.superstructure.HomingCommand;
import org.team4639.robot.commands.superstructure.SetpointSuperstructureCommand;
import org.team4639.robot.constants.Controls;
import org.team4639.robot.constants.FieldConstants;
import org.team4639.robot.robot.Subsystems;
import org.team4639.robot.subsystems.superstructure.Superstructure;
import org.team4639.robot.subsystems.superstructure.SuperstructureSetpoints;

public class SuperstructureCommands {
  public static Command IDLE;
  public static Command HP;
  public static Command HP_LOWER;
  public static Command CORAL_STOW;
  public static Command L4_PREP;
  public static Command L3_PREP;
  public static Command L2_PREP;
  public static Command L1_PREP;
  public static Command L1;
  public static Command L2;
  public static Command L3;
  public static Command L4;
  public static Command L2_ALGAE;
  public static Command L3_ALGAE;
  public static Command BARGE;
  public static Command BARGE_SCORE;
  public static Command BARGE_NO_OUTTAKE;
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
                () ->
                    new SetpointSuperstructureCommand(SuperstructureSetpoints.IDLE, "IDLE")
                        .withNone(),
                Set.of(
                    Subsystems.elevator,
                    Subsystems.wrist,
                    Subsystems.roller,
                    Subsystems.superstructure))
            .withName("IDLE");
    HP =
        Commands.defer(
                () ->
                    new SetpointSuperstructureCommand(SuperstructureSetpoints.HP, "HP")
                        .flashOnDone()
                        .withNone(),
                Set.of(
                    Subsystems.elevator,
                    Subsystems.wrist,
                    Subsystems.roller,
                    Subsystems.superstructure))
            .withName("HP");
    HP_LOWER =
        Commands.defer(
                () ->
                    new SetpointSuperstructureCommand(SuperstructureSetpoints.HP_LOWER, "HP_LOWER")
                        .withNone(),
                Set.of(
                    Subsystems.elevator,
                    Subsystems.wrist,
                    Subsystems.roller,
                    Subsystems.superstructure))
            .withName("HP_LOWER");
    CORAL_STOW =
        Commands.defer(
                () ->
                    new SetpointSuperstructureCommand(
                            SuperstructureSetpoints.CORAL_STOW, "CORAL_STOW")
                        .withCoral(),
                Set.of(
                    Subsystems.elevator,
                    Subsystems.wrist,
                    Subsystems.roller,
                    Subsystems.superstructure))
            .withName("CORAL_STOW");

    L4_PREP =
        Commands.defer(
                () ->
                    new SetpointSuperstructureCommand(SuperstructureSetpoints.L4_PREP, "L4_PREP")
                        .withCoral(),
                Set.of(
                    Subsystems.elevator,
                    Subsystems.wrist,
                    Subsystems.roller,
                    Subsystems.superstructure))
            .withName("L4_PREP");

    L3_PREP =
        Commands.defer(
                () ->
                    new SetpointSuperstructureCommand(SuperstructureSetpoints.L3_PREP, "L3_PREP")
                        .withCoral(),
                Set.of(
                    Subsystems.elevator,
                    Subsystems.wrist,
                    Subsystems.roller,
                    Subsystems.superstructure))
            .withName("L3_PREP");

    L2_PREP =
        Commands.defer(
                () ->
                    new SetpointSuperstructureCommand(SuperstructureSetpoints.L2_PREP, "L2_PREP")
                        .withCoral(),
                Set.of(
                    Subsystems.elevator,
                    Subsystems.wrist,
                    Subsystems.roller,
                    Subsystems.superstructure))
            .withName("L2_PREP");

    L1_PREP =
        Commands.defer(
                () ->
                    new SetpointSuperstructureCommand(SuperstructureSetpoints.L2_PREP, "L1_PREP")
                        .withCoral(),
                Set.of(
                    Subsystems.elevator,
                    Subsystems.wrist,
                    Subsystems.roller,
                    Subsystems.superstructure))
            .withName("L1_PREP");

    L1 =
        Commands.defer(
                () ->
                    new SetpointSuperstructureCommand(SuperstructureSetpoints.L1, "L1")
                        .flashOnDone()
                        .withCoral(),
                Set.of(
                    Subsystems.elevator,
                    Subsystems.wrist,
                    Subsystems.roller,
                    Subsystems.superstructure))
            .withName("L1");
    L2 =
        Commands.defer(
                () ->
                    new SetpointSuperstructureCommand(SuperstructureSetpoints.L2, "L2")
                        .flashOnDone()
                        .withCoral(),
                Set.of(
                    Subsystems.elevator,
                    Subsystems.wrist,
                    Subsystems.roller,
                    Subsystems.superstructure))
            .withName("L2");
    L3 =
        Commands.defer(
                () ->
                    new SetpointSuperstructureCommand(SuperstructureSetpoints.L3, "L3")
                        .flashOnDone()
                        .withCoral(),
                Set.of(
                    Subsystems.elevator,
                    Subsystems.wrist,
                    Subsystems.roller,
                    Subsystems.superstructure))
            .withName("L3");
    L4 =
        Commands.defer(
                () ->
                    new SetpointSuperstructureCommand(SuperstructureSetpoints.L4, "L4")
                        .flashOnDone()
                        .withCoral(),
                Set.of(
                    Subsystems.elevator,
                    Subsystems.wrist,
                    Subsystems.roller,
                    Subsystems.superstructure))
            .withName("L4");
    L2_ALGAE =
        Commands.defer(
                () ->
                    new SetpointSuperstructureCommand(SuperstructureSetpoints.L2_ALGAE, "L2_ALGAE")
                        .withAlgae(),
                Set.of(
                    Subsystems.elevator,
                    Subsystems.wrist,
                    Subsystems.roller,
                    Subsystems.superstructure))
            .withName("L2_ALGAE");
    L3_ALGAE =
        Commands.defer(
                () ->
                    new SetpointSuperstructureCommand(SuperstructureSetpoints.L3_ALGAE, "L3_ALGAE")
                        .withAlgae(),
                Set.of(
                    Subsystems.elevator,
                    Subsystems.wrist,
                    Subsystems.roller,
                    Subsystems.superstructure))
            .withName("L3_ALGAE");
    BARGE =
        Commands.defer(
                () ->
                    new SetpointSuperstructureCommand(SuperstructureSetpoints.BARGE, "BARGE")
                        .withAlgae(),
                Set.of(
                    Subsystems.elevator,
                    Subsystems.wrist,
                    Subsystems.roller,
                    Subsystems.superstructure))
            .withName("BARGE");

    BARGE_SCORE =
        Commands.defer(
                () ->
                    new SetpointSuperstructureCommand(SuperstructureSetpoints.BARGE_SCORE, "BARGE")
                        .withAlgae(),
                Set.of(
                    Subsystems.elevator,
                    Subsystems.wrist,
                    Subsystems.roller,
                    Subsystems.superstructure))
            .withName("BARGE");

    BARGE_NO_OUTTAKE =
        Commands.defer(
                () ->
                    new SetpointSuperstructureCommand(
                            SuperstructureSetpoints.BARGE_NO_OUTTAKE, "BARGE_NO_OUTTAKE")
                        .withAlgae(),
                Set.of(
                    Subsystems.elevator,
                    Subsystems.wrist,
                    Subsystems.roller,
                    Subsystems.superstructure))
            .withName("BARGE");

    ALGAE_STOW =
        Commands.defer(
                () ->
                    (new SetpointSuperstructureCommand(
                                SuperstructureSetpoints.ALGAE_STOW, "ALGAE_STOW")
                            .withAlgae())
                        .withTimeout(1)
                        .andThen(
                            (new SetpointSuperstructureCommand(
                                    SuperstructureSetpoints.ALGAE_STOW_LOWER, "ALGAE_STOW")
                                .withAlgae())),
                Set.of(
                    Subsystems.elevator,
                    Subsystems.wrist,
                    Subsystems.roller,
                    Subsystems.superstructure))
            .withName("ALGAE_STOW");
    HOMING_READY =
        Commands.defer(
                () ->
                    new SetpointSuperstructureCommand(
                            SuperstructureSetpoints.HOMING_READY, "HOMING_READY")
                        .withNone(),
                Set.of(
                    Subsystems.elevator,
                    Subsystems.wrist,
                    Subsystems.roller,
                    Subsystems.superstructure))
            .withName("HOMING_READY");
    HOMING =
        Commands.defer(
                HomingCommand::new,
                Set.of(
                    Subsystems.elevator,
                    Subsystems.wrist,
                    Subsystems.roller,
                    Subsystems.superstructure))
            .withName("HOMING");

    HOLD =
        Commands.defer(
                () ->
                    new SetpointSuperstructureCommand(
                        Superstructure.getSuperstructureState(), "HOLD"),
                Set.of(
                    Subsystems.superstructure,
                    Subsystems.elevator,
                    Subsystems.wrist,
                    Subsystems.roller))
            .withName("HOLD");

    REJECT_CORAL =
        Commands.defer(
                () ->
                    new SetpointSuperstructureCommand(
                        SuperstructureSetpoints.REJECT_CORAL, "REJECT_CORAL"),
                Set.of(
                    Subsystems.elevator,
                    Subsystems.wrist,
                    Subsystems.roller,
                    Subsystems.superstructure))
            .withName("REJECT_CORAL");
    REJECT_ALGAE =
        Commands.defer(
                () ->
                    new SetpointSuperstructureCommand(
                        SuperstructureSetpoints.REJECT_ALGAE, "REJECT_ALGAE"),
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
  public static SetpointSuperstructureCommand idle() {
    return new SetpointSuperstructureCommand(SuperstructureSetpoints.IDLE, "IDLE");
  }

  public static SetpointSuperstructureCommand hp() {
    return new SetpointSuperstructureCommand(SuperstructureSetpoints.HP, "HP").flashOnDone();
  }

  public static SetpointSuperstructureCommand hpLower() {
    return new SetpointSuperstructureCommand(SuperstructureSetpoints.HP_LOWER, "HP_LOWER")
        .flashOnDone();
  }

  public static SetpointSuperstructureCommand hpLowerAuto() {
    return new SetpointSuperstructureCommand(SuperstructureSetpoints.HP_LOWER_AUTO, "HP_LOWER")
        .flashOnDone();
  }

  public static SetpointSuperstructureCommand coralStow() {
    return new SetpointSuperstructureCommand(SuperstructureSetpoints.CORAL_STOW, "CORAL_STOW");
  }

  public static SetpointSuperstructureCommand elevatorReady() {
    return new SetpointSuperstructureCommand(SuperstructureSetpoints.L4_PREP, "L4_PREP");
  }

  public static SetpointSuperstructureCommand l1() {
    return new SetpointSuperstructureCommand(SuperstructureSetpoints.L1, "L1").flashOnDone();
  }

  public static SetpointSuperstructureCommand l2() {
    return new SetpointSuperstructureCommand(SuperstructureSetpoints.L2, "L2").flashOnDone();
  }

  public static SetpointSuperstructureCommand l3() {
    return new SetpointSuperstructureCommand(SuperstructureSetpoints.L3, "L3").flashOnDone();
  }

  public static Command l4() {
    return Commands.defer(
            () ->
                new SetpointSuperstructureCommand(SuperstructureSetpoints.L4, "L4")
                    .flashOnDone()
                    .withCoral(),
            Set.of(
                Subsystems.elevator,
                Subsystems.wrist,
                Subsystems.roller,
                Subsystems.superstructure))
        .withName("L4");
  }

  private static Command l4AutoCreator() {
    var command =
        new SetpointSuperstructureCommand(SuperstructureSetpoints.L4, "L4")
            .flashOnDone()
            .withCoral();
    return command.alongWith(Commands.waitSeconds(2).finallyDo(command::forceRoller));
  }

  public static Command l4Auto() {
    return Commands.defer(
            SuperstructureCommands::l4AutoCreator,
            Set.of(
                Subsystems.elevator,
                Subsystems.wrist,
                Subsystems.roller,
                Subsystems.superstructure))
        .withName("L4");
  }

  public static SetpointSuperstructureCommand l2Algae() {
    return new SetpointSuperstructureCommand(SuperstructureSetpoints.L2_ALGAE, "L2_ALGAE");
  }

  public static SetpointSuperstructureCommand l3Algae() {
    return new SetpointSuperstructureCommand(SuperstructureSetpoints.L3_ALGAE, "L3_ALGAE");
  }

  public static SetpointSuperstructureCommand barge() {
    return new SetpointSuperstructureCommand(SuperstructureSetpoints.BARGE, "BARGE")
        .flashOnDone()
        .waitForRoller(Controls.ALGAE_BARGE.or(Controls.ALGAE_BARGE_MANUAL));
  }

  public static SetpointSuperstructureCommand algaeStow() {
    return new SetpointSuperstructureCommand(SuperstructureSetpoints.ALGAE_STOW, "ALGAE_STOW");
  }

  public static SetpointSuperstructureCommand homingReady() {
    return new SetpointSuperstructureCommand(SuperstructureSetpoints.HOMING_READY, "HOMING_READY");
  }

  public static Command homing() {
    return Commands.defer(
            HomingCommand::new,
            Set.of(
                Subsystems.elevator,
                Subsystems.wrist,
                Subsystems.roller,
                Subsystems.superstructure))
        .withName("HOMING");
  }

  public static Command hold() {
    return Commands.defer(
        () -> new SetpointSuperstructureCommand(Superstructure.getSuperstructureState(), "HOLD"),
        Set.of(
            Subsystems.superstructure, Subsystems.elevator, Subsystems.wrist, Subsystems.roller));
  }

  public static SetpointSuperstructureCommand rejectCoral() {
    return new SetpointSuperstructureCommand(SuperstructureSetpoints.REJECT_CORAL, "REJECT_CORAL");
  }

  public static SetpointSuperstructureCommand rejectAlgae() {
    return new SetpointSuperstructureCommand(SuperstructureSetpoints.REJECT_ALGAE, "REJECT_ALGAE");
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

  public static Command l4Manual() {
    return Commands.defer(
            () ->
                new SetpointSuperstructureCommand(SuperstructureSetpoints.L4, "L4")
                    .flashOnDone()
                    .withCoral()
                    .waitForRoller(),
            Set.of(
                Subsystems.elevator,
                Subsystems.wrist,
                Subsystems.roller,
                Subsystems.superstructure))
        .withName("L4");
  }

  public static Command l3Manual() {
    return Commands.defer(
            () ->
                new SetpointSuperstructureCommand(SuperstructureSetpoints.L3, "L3")
                    .flashOnDone()
                    .withCoral()
                    .waitForRoller(),
            Set.of(
                Subsystems.elevator,
                Subsystems.wrist,
                Subsystems.roller,
                Subsystems.superstructure))
        .withName("L3");
  }

  public static Command l2Manual() {
    return Commands.defer(
            () ->
                new SetpointSuperstructureCommand(SuperstructureSetpoints.L2, "L2")
                    .flashOnDone()
                    .withCoral()
                    .waitForRoller(),
            Set.of(
                Subsystems.elevator,
                Subsystems.wrist,
                Subsystems.roller,
                Subsystems.superstructure))
        .withName("L2");
  }

  public static Command l1Manual() {
    return Commands.defer(
            () ->
                new SetpointSuperstructureCommand(SuperstructureSetpoints.L1, "L1")
                    .flashOnDone()
                    .withCoral()
                    .waitForRoller(),
            Set.of(
                Subsystems.elevator,
                Subsystems.wrist,
                Subsystems.roller,
                Subsystems.superstructure))
        .withName("L1");
  }

  public static Command autoL4Prep() {
    return Commands.defer(
            () ->
                new SetpointSuperstructureCommand(SuperstructureSetpoints.AUTO_L4_PREP, "L4_PREP")
                    .withCoral(),
            Set.of(
                Subsystems.elevator,
                Subsystems.wrist,
                Subsystems.roller,
                Subsystems.superstructure))
        .withName("L4_PREP");
  }
}
