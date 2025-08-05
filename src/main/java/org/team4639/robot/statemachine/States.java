package org.team4639.robot.statemachine;

import static edu.wpi.first.units.Units.Amps;
import static edu.wpi.first.units.Units.Seconds;
import static edu.wpi.first.units.Units.Volts;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Commands;
import org.team4639.lib.statebased.State;
import org.team4639.robot.commands.AutoCommands;
import org.team4639.robot.commands.DriveCommands;
import org.team4639.robot.commands.LEDCommands;
import org.team4639.robot.commands.SuperstructureCommands;
import org.team4639.robot.commands.superstructure.MicroAdjustmentCommand;
import org.team4639.robot.constants.Controls;
import org.team4639.robot.constants.FieldConstants;
import org.team4639.robot.modaltriggers.DriveTriggers;
import org.team4639.robot.robot.RobotContainer;
import org.team4639.robot.robot.Subsystems;
import org.team4639.robot.subsystems.superstructure.Superstructure;
import org.team4639.robot.subsystems.superstructure.SuperstructureSetpoints;

public class States {

  public static State IDLE;
  public static State NONE = new State("NONE");
  public static State HP_LEFT;
  public static State HP_RIGHT;
  public static State HP_NODIR;
  public static State INTAKE_LOWER;
  public static State CORAL_STOW;
  public static State ALIGN_ALGAE;
  public static State ALGAE_INTAKE;
  public static State CORAL_SCORE_ALIGN_LEFT;
  public static State CORAL_SCORE_ALIGN_RIGHT;
  public static State ALGAE_STOW;
  public static State ALGAE_SCORE;
  public static State CHOOSE_CORAL_LEVEL;
  public static State L1_CORAL_SCORE;
  public static State L2_CORAL_SCORE;
  public static State L3_CORAL_SCORE;
  public static State L4_CORAL_SCORE;
  public static State HOMING_READY;
  public static State HOMING;
  public static State REJECT_CORAL;
  public static State REJECT_ALGAE;
  public static State MICROADJUSTMENTS;

  public static State TEST_IDLE;
  public static State TEST_INTAKE;
  public static State TEST_L4;

  public static void initStaticStates() {
    IDLE =
        new State("IDLE")
            .whileTrue(SuperstructureCommands.IDLE)
            .onTrigger(DriveTriggers.closeToRightStation, () -> HP_RIGHT)
            .onTrigger(DriveTriggers.closeToLeftStation, () -> HP_LEFT)
            .withEndCondition(Controls.intake, () -> HP_NODIR)
            .withEndCondition(Subsystems.wrist::hasCoral, () -> CORAL_STOW)
            .withEndCondition(Controls.LEFT_HP, States::pathFindToHPLeft)
            .withEndCondition(Controls.RIGHT_HP, States::pathFindToHPRight)
            .withEndCondition(
                Controls.REEF_AB,
                () -> States.pathFindToReefAlgae(FieldConstants.TargetPositions.REEF_AB))
            .withEndCondition(
                Controls.REEF_CD,
                () -> States.pathFindToReefAlgae(FieldConstants.TargetPositions.REEF_CD))
            .withEndCondition(
                Controls.REEF_EF,
                () -> States.pathFindToReefAlgae(FieldConstants.TargetPositions.REEF_EF))
            .withEndCondition(
                Controls.REEF_GH,
                () -> States.pathFindToReefAlgae(FieldConstants.TargetPositions.REEF_GH))
            .withEndCondition(
                Controls.REEF_IJ,
                () -> States.pathFindToReefAlgae(FieldConstants.TargetPositions.REEF_IJ))
            .withEndCondition(
                Controls.REEF_KL,
                () -> States.pathFindToReefAlgae(FieldConstants.TargetPositions.REEF_KL));

    HP_LEFT =
        new State("HP_LEFT")
            .whileTrue(
                DriveCommands.coralStationAlignLeft(Subsystems.drive), SuperstructureCommands.HP)
            .withEndCondition(DriveTriggers.closeToLeftStation.negate(), () -> INTAKE_LOWER)
            .onTrigger(Controls.secondIntake, () -> INTAKE_LOWER)
            .onEmergency(() -> IDLE);

    HP_RIGHT =
        new State("HP_RIGHT")
            .whileTrue(
                DriveCommands.coralStationAlignRight(Subsystems.drive), SuperstructureCommands.HP)
            .withEndCondition(DriveTriggers.closeToRightStation.negate(), () -> INTAKE_LOWER)
            .onTrigger(Controls.secondIntake, () -> INTAKE_LOWER)
            .onEmergency(() -> IDLE);

    HP_NODIR =
        new State("INTAKE_NODIR")
            .whileTrue(SuperstructureCommands.HP)
            .onTrigger(Controls.secondIntake, () -> INTAKE_LOWER);

    INTAKE_LOWER =
        new State("INTAKE_LOWER")
            .whileTrue(SuperstructureCommands.HP_LOWER)
            .withEndCondition(Subsystems.wrist::hasCoral, () -> CORAL_STOW)
            .onEmergency(() -> IDLE);

    CORAL_STOW =
        new State("CORAL_STOW")
            .whileTrue(
                SuperstructureCommands.CORAL_STOW,
                DriveCommands.joystickDriveAtAngle(
                    Subsystems.drive,
                    () -> -RobotContainer.driver.getLeftY(),
                    () -> -RobotContainer.driver.getLeftX(),
                    () ->
                        FieldConstants.getRotationToClosestBranchPosition(
                            Subsystems.drive.getPose())),
                LEDCommands.hasCoral())
            .onTrigger(Controls.alignLeft, () -> CORAL_SCORE_ALIGN_LEFT)
            .onTrigger(Controls.alignRight, () -> CORAL_SCORE_ALIGN_RIGHT)
            .onTrigger(Controls.REEF_A, () -> pathFindToReef(FieldConstants.TargetPositions.REEF_A))
            .onTrigger(Controls.REEF_B, () -> pathFindToReef(FieldConstants.TargetPositions.REEF_B))
            .onTrigger(Controls.REEF_C, () -> pathFindToReef(FieldConstants.TargetPositions.REEF_C))
            .onTrigger(Controls.REEF_D, () -> pathFindToReef(FieldConstants.TargetPositions.REEF_D))
            .onTrigger(Controls.REEF_E, () -> pathFindToReef(FieldConstants.TargetPositions.REEF_E))
            .onTrigger(Controls.REEF_F, () -> pathFindToReef(FieldConstants.TargetPositions.REEF_F))
            .onTrigger(Controls.REEF_G, () -> pathFindToReef(FieldConstants.TargetPositions.REEF_G))
            .onTrigger(Controls.REEF_H, () -> pathFindToReef(FieldConstants.TargetPositions.REEF_H))
            .onTrigger(Controls.REEF_I, () -> pathFindToReef(FieldConstants.TargetPositions.REEF_I))
            .onTrigger(Controls.REEF_J, () -> pathFindToReef(FieldConstants.TargetPositions.REEF_J))
            .onTrigger(Controls.REEF_K, () -> pathFindToReef(FieldConstants.TargetPositions.REEF_K))
            .onTrigger(Controls.REEF_L, () -> pathFindToReef(FieldConstants.TargetPositions.REEF_L))
            .withEndCondition(Subsystems.wrist::doesNotHaveCoral, () -> IDLE)
            .onEmergency(() -> REJECT_CORAL);

    CORAL_SCORE_ALIGN_LEFT =
        new State("CORAL_SCORE_ALIGN_LEFT")
            .withDeadline(
                Subsystems.drive.defer(() -> DriveCommands.reefAlignLeft(Subsystems.drive)),
                () -> CHOOSE_CORAL_LEVEL)
            .whileTrue(
                SuperstructureCommands.ELEVATOR_READY,
                Subsystems.dashboardOutputs.displayUpcomingReefLevel(),
                LEDCommands.aligning())
            .onTrigger(Controls.alignRight, () -> States.CORAL_SCORE_ALIGN_RIGHT)
            .onTrigger(Controls.REEF_A, () -> pathFindToReef(FieldConstants.TargetPositions.REEF_A))
            .onTrigger(Controls.REEF_B, () -> pathFindToReef(FieldConstants.TargetPositions.REEF_B))
            .onTrigger(Controls.REEF_C, () -> pathFindToReef(FieldConstants.TargetPositions.REEF_C))
            .onTrigger(Controls.REEF_D, () -> pathFindToReef(FieldConstants.TargetPositions.REEF_D))
            .onTrigger(Controls.REEF_E, () -> pathFindToReef(FieldConstants.TargetPositions.REEF_E))
            .onTrigger(Controls.REEF_F, () -> pathFindToReef(FieldConstants.TargetPositions.REEF_F))
            .onTrigger(Controls.REEF_G, () -> pathFindToReef(FieldConstants.TargetPositions.REEF_G))
            .onTrigger(Controls.REEF_H, () -> pathFindToReef(FieldConstants.TargetPositions.REEF_H))
            .onTrigger(Controls.REEF_I, () -> pathFindToReef(FieldConstants.TargetPositions.REEF_I))
            .onTrigger(Controls.REEF_J, () -> pathFindToReef(FieldConstants.TargetPositions.REEF_J))
            .onTrigger(Controls.REEF_K, () -> pathFindToReef(FieldConstants.TargetPositions.REEF_K))
            .onTrigger(Controls.REEF_L, () -> pathFindToReef(FieldConstants.TargetPositions.REEF_L))
            .withEndCondition(Subsystems.wrist::doesNotHaveCoral, () -> IDLE)
            .onEmergency(() -> CORAL_STOW)
            .onAccelerationLimit(() -> CORAL_STOW);

    CORAL_SCORE_ALIGN_RIGHT =
        new State("CORAL_SCORE_ALIGN_RIGHT")
            .withDeadline(
                Subsystems.drive.defer(() -> DriveCommands.reefAlignRight(Subsystems.drive)),
                () -> CHOOSE_CORAL_LEVEL)
            .whileTrue(
                SuperstructureCommands.ELEVATOR_READY,
                Subsystems.dashboardOutputs.displayUpcomingReefLevel(),
                LEDCommands.aligning())
            .onTrigger(Controls.alignLeft, () -> States.CORAL_SCORE_ALIGN_LEFT)
            .onTrigger(Controls.REEF_A, () -> pathFindToReef(FieldConstants.TargetPositions.REEF_A))
            .onTrigger(Controls.REEF_B, () -> pathFindToReef(FieldConstants.TargetPositions.REEF_B))
            .onTrigger(Controls.REEF_C, () -> pathFindToReef(FieldConstants.TargetPositions.REEF_C))
            .onTrigger(Controls.REEF_D, () -> pathFindToReef(FieldConstants.TargetPositions.REEF_D))
            .onTrigger(Controls.REEF_E, () -> pathFindToReef(FieldConstants.TargetPositions.REEF_E))
            .onTrigger(Controls.REEF_F, () -> pathFindToReef(FieldConstants.TargetPositions.REEF_F))
            .onTrigger(Controls.REEF_G, () -> pathFindToReef(FieldConstants.TargetPositions.REEF_G))
            .onTrigger(Controls.REEF_H, () -> pathFindToReef(FieldConstants.TargetPositions.REEF_H))
            .onTrigger(Controls.REEF_I, () -> pathFindToReef(FieldConstants.TargetPositions.REEF_I))
            .onTrigger(Controls.REEF_J, () -> pathFindToReef(FieldConstants.TargetPositions.REEF_J))
            .onTrigger(Controls.REEF_K, () -> pathFindToReef(FieldConstants.TargetPositions.REEF_K))
            .onTrigger(Controls.REEF_L, () -> pathFindToReef(FieldConstants.TargetPositions.REEF_L))
            .withEndCondition(Subsystems.wrist::doesNotHaveCoral, () -> IDLE)
            .onEmergency(() -> CORAL_STOW)
            .onAccelerationLimit(() -> CORAL_STOW);

    ALIGN_ALGAE =
        new State("ALIGN_ALGAE")
            .withDeadline(
                Subsystems.drive.defer(() -> DriveCommands.reefAlign(Subsystems.drive)),
                () -> ALGAE_INTAKE)
            .whileTrue(SuperstructureCommands.ALGAE_INTAKE)
            .onEmergency(() -> IDLE)
            .onAccelerationLimit(() -> IDLE);

    ALGAE_INTAKE =
        new State("ALGAE_INTAKE")
            .withDeadline(AutoCommands.algaeIntakeSequence(), () -> ALGAE_STOW)
            .onEmergency(() -> IDLE)
            .onAccelerationLimit(() -> IDLE);

    ALGAE_STOW =
        new State("ALGAE_STOW")
            .whileTrue(SuperstructureCommands.ALGAE_STOW)
            .onEmergency(() -> REJECT_ALGAE);

    ALGAE_SCORE =
        new State("ALGAE_SCORE")
            .whileTrue(SuperstructureCommands.BARGE)
            .onEmergency(() -> ALGAE_STOW)
            .onAccelerationLimit(() -> ALGAE_STOW);

    CHOOSE_CORAL_LEVEL =
        new State("CHOOSE_CORAL_LEVEL")
            .whileTrue(
                SuperstructureCommands.HOLD, Subsystems.drive.run(() -> Subsystems.drive.stop()))
            .withEndCondition(Controls.alignLeft, () -> CORAL_SCORE_ALIGN_LEFT)
            .withEndCondition(Controls.alignRight, () -> CORAL_SCORE_ALIGN_RIGHT)
            .withEndCondition(Subsystems.wrist::doesNotHaveCoral, () -> IDLE)
            .withEndCondition(
                () -> Subsystems.dashboardOutputs.upcomingReefLevel() == 1, () -> L1_CORAL_SCORE)
            .withEndCondition(
                () -> Subsystems.dashboardOutputs.upcomingReefLevel() == 2, () -> L2_CORAL_SCORE)
            .withEndCondition(
                () -> Subsystems.dashboardOutputs.upcomingReefLevel() == 3, () -> L3_CORAL_SCORE)
            .withEndCondition(
                () -> Subsystems.dashboardOutputs.upcomingReefLevel() == 4, () -> L4_CORAL_SCORE);

    L1_CORAL_SCORE =
        new State("L1_CORAL_SCORE")
            .whileTrue(SuperstructureCommands.L1)
            .withEndCondition(Subsystems.wrist::doesNotHaveCoral, () -> HOMING_READY)
            .onEmergency(() -> CORAL_STOW)
            .onAccelerationLimit(() -> CORAL_STOW);

    L2_CORAL_SCORE =
        new State("L2_CORAL_SCORE")
            .whileTrue(SuperstructureCommands.L2)
            .withEndCondition(Subsystems.wrist::doesNotHaveCoral, () -> HOMING_READY)
            .onEmergency(() -> CORAL_STOW)
            .onAccelerationLimit(() -> CORAL_STOW);

    L3_CORAL_SCORE =
        new State("L3_CORAL_SCORE")
            .whileTrue(SuperstructureCommands.L3)
            .withEndCondition(Subsystems.wrist::doesNotHaveCoral, () -> HOMING_READY)
            .onEmergency(() -> CORAL_STOW)
            .onAccelerationLimit(() -> CORAL_STOW);

    L4_CORAL_SCORE =
        new State("L4_CORAL_SCORE")
            .whileTrue(SuperstructureCommands.L4)
            .withEndCondition(Subsystems.wrist::doesNotHaveCoral, () -> HOMING_READY)
            .onEmergency(() -> CORAL_STOW)
            .onAccelerationLimit(() -> CORAL_STOW);

    HOMING_READY =
        new State("HOMING_READY")
            .whileTrue(SuperstructureCommands.HOMING_READY)
            .withEndCondition(
                () ->
                    Superstructure.atPosition(
                        Superstructure.getSuperstructureState(),
                        SuperstructureSetpoints.HOMING_READY),
                () -> HOMING)
            .onEmergency(() -> IDLE);

    HOMING =
        new State("HOMING")
            .withDeadline(SuperstructureCommands.HOMING, () -> IDLE)
            .onEmergency(() -> IDLE);

    REJECT_CORAL =
        new State("REJECT_CORAL")
            .whileTrue(SuperstructureCommands.REJECT_CORAL)
            .withEndCondition(Subsystems.wrist::doesNotHaveCoral, () -> IDLE);

    REJECT_ALGAE =
        new State("REJECT_ALGAE")
            .whileTrue(SuperstructureCommands.REJECT_ALGAE)
            .withTimeout(Seconds.of(0.5), () -> IDLE);

    MICROADJUSTMENTS =
        new State("MICROADJUSTMENTS")
            .whileTrue(new MicroAdjustmentCommand(), DriveCommands.stopWithX())
            .onEmergency(() -> CORAL_STOW);

    TEST_IDLE = new State("TEST_IDLE");

    /*TEST_IDLE
        .trigger()
        .and(RobotContainer.driver.a())
        .whileTrue(SuperstructureCommands.ELEVATOR_READY);
    TEST_IDLE.trigger().and(RobotContainer.driver.b()).whileTrue(SuperstructureCommands.L4);*/
    SmartDashboard.putNumber("Roller Voltage", 0);

    TEST_IDLE
        .trigger()
        .and(RobotContainer.driver.a())
        .onTrue(
            Commands.run(() -> Subsystems.roller.setVoltage(Volts.of(-5)))
                .until(() -> Subsystems.roller.getInputs().motorCurrent.gte(Amps.of(78)))
                .andThen(Commands.run(() -> Subsystems.roller.setVoltage(Volts.of(-1)))));
  }

  public static State pathFindToReef(FieldConstants.TargetPositions reef) {
    var pose = reef.getPose();
    return new State("PATHFIND_TO_REEF")
        .withDeadline(
            DriveCommands.pathFindToReef(Subsystems.drive, pose), () -> CHOOSE_CORAL_LEVEL)
        .whileTrue(
            SuperstructureCommands.ELEVATOR_READY,
            Subsystems.dashboardOutputs.displayUpcomingReefLevel())
        .withEndCondition(Subsystems.wrist::doesNotHaveCoral, () -> IDLE)
        .onEmergency(() -> CORAL_STOW);
  }

  public static State pathFindToHPLeft() {
    return new State("PATHFIND_TO_HP_LEFT")
        .withDeadline(
            DriveCommands.pathFindToHP(
                Subsystems.drive, FieldConstants.TargetPositions.CORALSTATION_LEFT.getPose()),
            () -> HP_LEFT)
        .whileTrue(SuperstructureCommands.HP)
        .onEmergency(() -> IDLE);
  }

  public static State pathFindToHPRight() {
    return new State("PATHFIND_TO_HP_RIGHT")
        .withDeadline(
            DriveCommands.pathFindToHP(
                Subsystems.drive, FieldConstants.TargetPositions.CORALSTATION_RIGHT.getPose()),
            () -> HP_RIGHT)
        .whileTrue(SuperstructureCommands.HP)
        .onEmergency(() -> IDLE);
  }

  public static State pathFindToReefAlgae(FieldConstants.TargetPositions reefCenter) {
    return new State("PATHFIND_TO_REEF_CENTER")
        .withDeadline(
            DriveCommands.pathFindToReefCenter(Subsystems.drive, reefCenter.getPose()),
            () -> ALGAE_INTAKE)
        .whileTrue(SuperstructureCommands.algaeIntake(reefCenter.getPose()))
        .onEmergency(() -> IDLE);
  }

  public static State determineState() {
    return switch (SmartDashboard.getString("Superstructure State", "")) {
      case "IDLE" -> IDLE;
      case "HP" -> HP_NODIR;
      case "HP_LOWER" -> INTAKE_LOWER;
      case "L4" -> L4_CORAL_SCORE;
      default -> Subsystems.wrist.hasCoral() ? CORAL_STOW : IDLE;
    };
  }
}
