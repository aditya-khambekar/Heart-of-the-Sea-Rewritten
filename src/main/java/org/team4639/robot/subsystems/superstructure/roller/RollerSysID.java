package org.team4639.robot.subsystems.superstructure.roller;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine.Direction;
import org.team4639.lib.command.Commands2;
import org.team4639.robot.robot.Subsystems;

public class RollerSysID {

  public void init() {
    routine =
        new SysIdRoutine(
            new SysIdRoutine.Config(),
            new SysIdRoutine.Mechanism(
                Subsystems.roller::setVoltage,
                log ->
                    log.motor("roller")
                        .angularVelocity(Subsystems.roller.getInputs().motorVelocity)
                        .angularPosition(Subsystems.roller.getInputs().motorPosition)
                        .voltage(Subsystems.roller.getInputs().voltage)
                        .current(Subsystems.roller.getInputs().motorCurrent),
                Subsystems.roller));
  }

  private static SysIdRoutine routine;

  public Command kQuasistaticForward() {
    return Commands2.defer(() -> routine.quasistatic(Direction.kForward));
  }

  public Command kQuasistaticReverse() {
    return Commands2.defer(() -> routine.quasistatic(Direction.kReverse));
  }

  public Command kDynamicForward() {
    return Commands2.defer(() -> routine.dynamic(Direction.kForward));
  }

  public Command kDynamicReverse() {
    return Commands2.defer(() -> routine.dynamic(Direction.kReverse));
  }
}
