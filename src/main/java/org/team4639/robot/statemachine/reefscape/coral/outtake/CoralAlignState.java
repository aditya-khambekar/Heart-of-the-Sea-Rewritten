package org.team4639.robot.statemachine.reefscape.coral.outtake;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import java.util.Map;
import org.team4639.robot.commands.LEDCommands;
import org.team4639.robot.commands.SuperstructureCommands;
import org.team4639.robot.robot.Subsystems;
import org.team4639.robot.statemachine.States;
import org.team4639.robot.statemachine.reefscape.coral.CoralCycleState;
import org.team4639.robot.subsystems.DashboardOutputs;

public class CoralAlignState extends CoralCycleState {
  private static final Map<Trigger, Command> dashboardOutputToSuperstructurePrep =
      Map.ofEntries(
          Map.entry(DashboardOutputs.getInstance().getSelectedL4(), SuperstructureCommands.L4_PREP),
          Map.entry(DashboardOutputs.getInstance().getSelectedL3(), SuperstructureCommands.L3_PREP),
          Map.entry(DashboardOutputs.getInstance().getSelectedL2(), SuperstructureCommands.L2_PREP),
          Map.entry(
              DashboardOutputs.getInstance().getSelectedL1(), SuperstructureCommands.L1_PREP));

  public CoralAlignState(String name) {
    super(name);
    this.mapTriggerCommandsWhileTrue(dashboardOutputToSuperstructurePrep);
    this.onEmergency(() -> States.CORAL_STOW);
    this.whileTrue(
        SuperstructureCommands.CORAL_STOW,
        DashboardOutputs.getInstance().displayUpcomingReefLevel(),
        LEDCommands.aligning());
    this.withEndCondition(Subsystems.wrist::doesNotHaveCoral, () -> States.IDLE);
  }
}
