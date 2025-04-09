package org.team4639.subsystems.scoring;

import edu.wpi.first.math.filter.Debouncer;
import edu.wpi.first.math.filter.Debouncer.DebounceType;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class Scoring extends SubsystemBase {
  private final ScoringIOInputsAutoLogged inputs;
  private final ScoringIO io;

  private final Debouncer coralCurrentLimitDebouncer;
  private final Debouncer algaeCurrentLimitDebouncer;

  public Scoring(ScoringIO io) {
    this.inputs = new ScoringIOInputsAutoLogged();
    this.io = io;

    coralCurrentLimitDebouncer = new Debouncer(0.1, DebounceType.kBoth);
    algaeCurrentLimitDebouncer = new Debouncer(0.1, DebounceType.kBoth);
  }

  public Command runMotor(double speed) {
    return run(() -> io.runSparkMax(speed));
  }

  public Command intakeCoral() {
    return runMotor(ScoringConstants.Speeds.CORAL_INTAKE_SPEED)
        .until(
            () ->
                coralCurrentLimitDebouncer.calculate(
                    getCurrent() > ScoringConstants.Currents.CORAL_INTAKE_CURRENT));
  }

  public Command outtakeCoral() {
    return runMotor(ScoringConstants.Speeds.CORAL_OUTTAKE_SPEED).until(() -> !hasCoral());
  }

  public Command intakeAlgae() {
    return runMotor(ScoringConstants.Speeds.ALGAE_INTAKE_SPEED)
        .until(
            () ->
                algaeCurrentLimitDebouncer.calculate(
                    getCurrent() > ScoringConstants.Currents.ALGAE_INTAKE_CURRENT));
  }

  public double getCurrent() {
    return inputs.currentAmps;
  }

  public boolean hasCoral() {
    return inputs.hasCoral;
  }

  @Override
  public void periodic() {
    io.updateInputs(inputs);
  }
}
