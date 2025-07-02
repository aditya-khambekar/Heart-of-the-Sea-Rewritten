package org.team4639.robot.commands.superstructure;

import static edu.wpi.first.units.Units.RotationsPerSecond;
import static edu.wpi.first.units.Units.Value;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.filter.Debouncer;
import edu.wpi.first.wpilibj.RobotBase;
import org.team4639.robot.robot.Subsystems;

public class HomingCommand extends SuperstructureCommandBase {
  private Debouncer stoppedDebouncer = new Debouncer(0.1, Debouncer.DebounceType.kRising);

  public HomingCommand() {
    addRequirements(
        Subsystems.elevator, Subsystems.wrist, Subsystems.roller, Subsystems.superstructure);
  }

  @Override
  public void execute() {
    super.execute();
    Subsystems.elevator.elevatorDown();
  }

  @Override
  public boolean isFinished() {
    return stoppedDebouncer.calculate(
        RobotBase.isReal()
            ? MathUtil.isNear(0, Subsystems.elevator.getMotorSpeed().in(RotationsPerSecond), 0.1)
            : Subsystems.elevator.getPercentage().in(Value) <= 0);
  }

  @Override
  public void end(boolean interrupted) {
    Subsystems.elevator.zero();
  }

  @Override
  public SuperstructureCommandState getState() {
    return SuperstructureCommandState.HOMING;
  }

  @Override
  public String getName() {
    return "HOMING";
  }
}
