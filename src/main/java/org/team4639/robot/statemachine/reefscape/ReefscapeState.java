package org.team4639.robot.statemachine.reefscape;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import java.util.Map;
import org.team4639.lib.statebased.State;
import org.team4639.robot.commands.SuperstructureCommands;
import org.team4639.robot.constants.Controls;
import org.team4639.robot.statemachine.States;

public class ReefscapeState extends State {
  private static final Map<Trigger, Command> operatorControls =
      Map.ofEntries(
          Map.entry(Controls.L4_CORAL_MANUAL, SuperstructureCommands.l4Manual()),
          Map.entry(Controls.L3_CORAL_MANUAL, SuperstructureCommands.l3Manual()),
          Map.entry(Controls.L2_CORAL_MANUAL, SuperstructureCommands.l2Manual()),
          Map.entry(Controls.L1_CORAL_MANUAL, SuperstructureCommands.l1Manual()),
          Map.entry(Controls.ALGAE_BARGE_MANUAL, SuperstructureCommands.barge()),
          Map.entry(Controls.INTAKE, SuperstructureCommands.hpLower()),
          Map.entry(Controls.ALGAE_INTAKE_HIGH, SuperstructureCommands.l3Algae()),
          Map.entry(Controls.ALGAE_INTAKE_LOW, SuperstructureCommands.l2Algae()));

  public ReefscapeState(String name) {
    super(name);
    this.mapTriggerCommandsOnTrue(operatorControls);
    this.onTrigger(Controls.FORCE_HOMING, () -> States.HOMING_READY);
  }
}
