package org.team4639.commands;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import org.team4639._robot.Subsystems;
import org.team4639.subsystems.elevator.ElevatorConstants;

public class SuperstructureCommands {
  public Command intakeCoral() {
    return Commands.sequence(
        Subsystems.elevator
            .runToSetpoint(
                ElevatorConstants.ProportionToPosition.convert(
                    ElevatorConstants.Setpoints.HP_Proportion))
            .until(Subsystems.elevator::atPosition),
        Subsystems.scoring.intakeCoral());
  }
}
