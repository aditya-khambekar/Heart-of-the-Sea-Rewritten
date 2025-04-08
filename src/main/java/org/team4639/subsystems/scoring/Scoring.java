package org.team4639.subsystems.scoring;

import org.littletonrobotics.junction.AutoLog;
import org.team4639.constants.IDs;
import org.team4639.subsystems.scoring.ScoringIO.ScoringIOInputs;

import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.SparkLowLevel.MotorType;

import edu.wpi.first.math.filter.Debouncer;
import edu.wpi.first.math.filter.Debouncer.DebounceType;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class Scoring extends SubsystemBase{
    //TODO: add LaserCAN here

    private final SparkMax scoringMotor;
    private final ScoringIOInputsAutoLogged inputs;
    private final ScoringIO io;

    private final Debouncer coralCurrentLimitDebouncer;
    private final Debouncer algaeCurrentLimitDebouncer;

    public Scoring(ScoringIO io){
        scoringMotor = new SparkMax(IDs.SCORING_MOTOR, MotorType.kBrushless);
        this.inputs = new ScoringIOInputsAutoLogged();
        this.io = io;
        io.sendSparkMaxData(scoringMotor);

        coralCurrentLimitDebouncer = new Debouncer(0.1, DebounceType.kBoth);
    }

    public Command runMotor(double speed) {
        return run(() -> scoringMotor.set(speed));
    }

    public Command intakeCoral() {
        return runMotor(ScoringConstants.Speeds.CORAL_INTAKE_SPEED)
        .until(() -> coralCurrentLimitDebouncer.calculate(getCurrent() > ScoringConstants.Currents.CORAL_INTAKE_CURRENT));
    }

    public Command intakeAlgae() {
        return runMotor(ScoringConstants.Speeds.ALGAE_INTAKE_SPEED)
        .until(() -> algaeCurrentLimitDebouncer.calculate(getCurrent() > ScoringConstants.Currents.ALGAE_INTAKE_CURRENT));
    }

    public double getCurrent() {
        return inputs.currentAmps;
    }

    @Override
    public void periodic() {
        io.updateInputs(inputs);
    }
}
