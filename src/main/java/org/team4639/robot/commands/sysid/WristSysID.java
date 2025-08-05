package org.team4639.robot.commands.sysid;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine.Direction;
import org.team4639.robot.robot.Subsystems;

public class WristSysID {
  private static SysIdRoutine routine =
      new SysIdRoutine(
          new SysIdRoutine.Config(),
          new SysIdRoutine.Mechanism(
              Subsystems.wrist::setVoltage,
              log -> {
                log.motor("wrist")
                    .angularPosition(Subsystems.wrist.getEncoderPosition())
                    .angularVelocity(Subsystems.wrist.getEncoderVelocity())
                    .voltage(Subsystems.wrist.getBusVoltage());
              },
              Subsystems.wrist));

  public static Command quasistaticForward() {
    return routine.quasistatic(Direction.kForward);
  }

  public static Command quasistaticReverse() {
    return routine.quasistatic(Direction.kReverse);
  }

  public static Command dynamicForward() {
    return routine.dynamic(Direction.kForward);
  }

  public static Command dynamicReverse() {
    return routine.dynamic(Direction.kReverse);
  }
}
