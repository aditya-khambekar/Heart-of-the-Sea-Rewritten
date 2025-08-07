package org.team4639.robot.statemachine;

import static edu.wpi.first.units.Units.Amps;
import static edu.wpi.first.units.Units.Seconds;
import static edu.wpi.first.units.Units.Volts;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import org.team4639.lib.statebased.State;
import org.team4639.robot.commands.AutoCommands;
import org.team4639.robot.commands.DriveCommands;
import org.team4639.robot.commands.LEDCommands;
import org.team4639.robot.commands.SuperstructureCommands;
import org.team4639.robot.constants.Controls;
import org.team4639.robot.constants.FieldConstants;
import org.team4639.robot.modaltriggers.DriveTriggers;
import org.team4639.robot.robot.RobotContainer;
import org.team4639.robot.robot.Subsystems;
import org.team4639.robot.subsystems.DashboardOutputs;
import org.team4639.robot.subsystems.superstructure.Superstructure;
import org.team4639.robot.subsystems.superstructure.SuperstructureSetpoints;

import java.util.Map;

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

  public static State TEST_IDLE;
  public static State TEST_INTAKE;
  public static State TEST_L4;

  private static Map<Trigger, Command> dashboardOutputToSuperstructurePrep =
          Map.ofEntries(
                  Map.entry(DashboardOutputs.getInstance().getSelectedL4(), SuperstructureCommands.L4_PREP),
                  Map.entry(DashboardOutputs.getInstance().getSelectedL3(), SuperstructureCommands.L3_PREP),
                  Map.entry(DashboardOutputs.getInstance().getSelectedL2(), SuperstructureCommands.L2_PREP),
                  Map.entry(DashboardOutputs.getInstance().getSelectedL1(), SuperstructureCommands.L1_PREP)
          );

  private static Map<Trigger, Command> driverInputToDashboardOutputSelector =
          Map.ofEntries(
                  Map.entry(Controls.L4_CORAL_SCORE, DashboardOutputs.getInstance().selectL4()),
                  Map.entry(Controls.L3_CORAL_SCORE, DashboardOutputs.getInstance().selectL3()),
                  Map.entry(Controls.L2_CORAL_SCORE, DashboardOutputs.getInstance().selectL2()),
                  Map.entry(Controls.L1_CORAL_SCORE, DashboardOutputs.getInstance().selectL1())
          );

  public static void initStaticStates() {
    IDLE =
        new State("IDLE")
            .whileTrue(SuperstructureCommands.IDLE)
            .onTrigger(DriveTriggers.closeToRightStation, () -> HP_RIGHT)
            .onTrigger(DriveTriggers.closeToLeftStation, () -> HP_LEFT)
            .withEndCondition(Subsystems.wrist::hasCoral, () -> CORAL_STOW);

    HP_LEFT =
        new State("HP_LEFT")
            .whileTrue(DriveCommands.HPLeftAlign(Subsystems.drive), SuperstructureCommands.HP)
            .withEndCondition(DriveTriggers.closeToLeftStation.negate(), () -> INTAKE_LOWER)
            .onEmergency(() -> IDLE);

    HP_RIGHT =
        new State("HP_RIGHT")
            .whileTrue(DriveCommands.HPRightAlign(Subsystems.drive), SuperstructureCommands.HP)
            .withEndCondition(DriveTriggers.closeToRightStation.negate(), () -> INTAKE_LOWER)
            .onEmergency(() -> IDLE);

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
                    () -> -RobotContainer.driver.getLeftY(),
                    () -> -RobotContainer.driver.getLeftX(),
                    () ->
                        FieldConstants.getRotationToClosestBranchPosition(
                            Subsystems.drive.getPose())),
                LEDCommands.hasCoral())
            .onTrigger(Controls.ALIGN_LEFT, () -> CORAL_SCORE_ALIGN_LEFT)
            .onTrigger(Controls.ALIGN_RIGHT, () -> CORAL_SCORE_ALIGN_RIGHT)
            .withEndCondition(Subsystems.wrist::doesNotHaveCoral, () -> IDLE)
            .onEmergency(() -> REJECT_CORAL);

    CORAL_SCORE_ALIGN_LEFT =
        new State("CORAL_SCORE_ALIGN_LEFT")
            .withDeadline(
                Subsystems.drive.defer(DriveCommands::alignToNearestReefLeft),
                () -> CHOOSE_CORAL_LEVEL)
            .whileTrue(
                SuperstructureCommands.CORAL_STOW,
                DashboardOutputs.getInstance().displayUpcomingReefLevel(),
                LEDCommands.aligning())
            .onTrigger(Controls.ALIGN_RIGHT, () -> States.CORAL_SCORE_ALIGN_RIGHT)
            .withEndCondition(Subsystems.wrist::doesNotHaveCoral, () -> IDLE)
            .onEmergency(() -> CORAL_STOW)
            .onAccelerationLimit(() -> CORAL_STOW)
                .mapTriggerCommandsWhileTrue(dashboardOutputToSuperstructurePrep)
                .mapTriggerCommandsOnTrue(driverInputToDashboardOutputSelector);

    CORAL_SCORE_ALIGN_RIGHT =
        new State("CORAL_SCORE_ALIGN_RIGHT")
            .withDeadline(
                Subsystems.drive.defer(DriveCommands::alignToNearestReefRight),
                () -> CHOOSE_CORAL_LEVEL)
            .whileTrue(
                    SuperstructureCommands.CORAL_STOW,
                DashboardOutputs.getInstance().displayUpcomingReefLevel(),
                LEDCommands.aligning())
            .onTrigger(Controls.ALIGN_LEFT, () -> States.CORAL_SCORE_ALIGN_LEFT)
            .withEndCondition(Subsystems.wrist::doesNotHaveCoral, () -> IDLE)
            .onEmergency(() -> CORAL_STOW)
            .onAccelerationLimit(() -> CORAL_STOW)
                .mapTriggerCommandsWhileTrue(dashboardOutputToSuperstructurePrep)
                .mapTriggerCommandsOnTrue(driverInputToDashboardOutputSelector);

    ALIGN_ALGAE =
        new State("ALIGN_ALGAE")
            .withDeadline(
                Subsystems.drive.defer(DriveCommands::alignToNearestReef),
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
            .withEndCondition(Controls.ALIGN_LEFT, () -> CORAL_SCORE_ALIGN_LEFT)
            .withEndCondition(Controls.ALIGN_RIGHT, () -> CORAL_SCORE_ALIGN_RIGHT)
            .withEndCondition(Subsystems.wrist::doesNotHaveCoral, () -> IDLE)
            .withEndCondition(
                () -> DashboardOutputs.getInstance().getUpcomingReefLevel() == 1, () -> L1_CORAL_SCORE)
            .withEndCondition(
                () -> DashboardOutputs.getInstance().getUpcomingReefLevel() == 2, () -> L2_CORAL_SCORE)
            .withEndCondition(
                () -> DashboardOutputs.getInstance().getUpcomingReefLevel() == 3, () -> L3_CORAL_SCORE)
            .withEndCondition(
                () -> DashboardOutputs.getInstance().getUpcomingReefLevel() == 4,
                () -> L4_CORAL_SCORE);

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

    TEST_IDLE = new State("TEST_IDLE");

    /*TEST_IDLE
        .trigger()
        .and(RobotContainer.driver.a())
        .whileTrue(SuperstructureCommands.L4_PREP);
    TEST_IDLE.trigger().and(RobotContainer.driver.b()).whileTrue(SuperstructureCommands.L4);*/

    TEST_IDLE
        .trigger()
        .and(RobotContainer.driver.a())
        .onTrue(
            Commands.run(() -> Subsystems.roller.setVoltage(Volts.of(-5)))
                .until(() -> Subsystems.roller.getInputs().motorCurrent.gte(Amps.of(78)))
                .andThen(Commands.run(() -> Subsystems.roller.setVoltage(Volts.of(-1)))));
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
