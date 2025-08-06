package org.team4639.lib.statebased;

import static edu.wpi.first.units.Units.Seconds;

import edu.wpi.first.units.measure.MutTime;
import edu.wpi.first.wpilibj.Notifier;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Commands;
import java.util.Objects;
import org.team4639.robot.constants.Controls;
import org.team4639.robot.statemachine.States;

public class StateMachine {
  private static volatile StateMachine instance;

  public static synchronized StateMachine getInstance() {
    return instance = Objects.requireNonNullElseGet(instance, StateMachine::new);
  }

  private State state;
  private final MutTime timeOfLastStateChange;

  private StateMachine() {
    state = States.NONE;
    timeOfLastStateChange = Seconds.mutable(0);
    Controls.emergency.onTrue(Commands.runOnce(() -> setState(state.onEmergency.get())));

    new Notifier(this::periodic).startPeriodic(0.1);
  }

  public static State getState() {
    return getInstance().state;
  }

  public static void setState(State newState) {
    if (newState != getInstance().state) {
      getInstance().state.cancelCommands();
      getInstance().state = newState;
      getInstance().state.scheduleCommands();
      getInstance().timeOfLastStateChange.mut_replace(Seconds.of(Timer.getFPGATimestamp()));
    }
  }

  public void periodic() {
    SmartDashboard.putString("State", state.getName());
    state.evaluate(Timer.getTimestamp() - timeOfLastStateChange.in(Seconds));
  }
}
