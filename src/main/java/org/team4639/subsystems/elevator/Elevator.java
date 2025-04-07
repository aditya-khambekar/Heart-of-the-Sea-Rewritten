package org.team4639.subsystems.elevator;
import com.ctre.phoenix6.controls.Follower;
import com.ctre.phoenix6.controls.MotionMagicVoltage;
import constants.IDs;
import edu.wpi.first.math.controller.ElevatorFeedforward;
import edu.wpi.first.util.sendable.Sendable;
import edu.wpi.first.util.sendable.SendableBuilder;
import org.team4639.lib.motorcontrol.generic.NeutralMode;
import org.team4639.lib.motorcontrol.talonfx.RSTalonFX;
import org.team4639.lib.motorcontrol.talonfx.RSTalonFXTemplate;
import org.team4639.lib.subsystem.RSSubsystem;

import static constants.IDs.*;

public class Elevator extends RSSubsystem implements Sendable {
    private final RSTalonFX leftMotor;
    private final RSTalonFX rightMotor;

    private final ElevatorFeedforward feedforward;

    public Elevator() {
        this.feedforward = new ElevatorFeedforward(ElevatorConstants.Params.ks, ElevatorConstants.Params.kg, ElevatorConstants.Params.kv, ElevatorConstants.Params.ka);
        leftMotor = RSTalonFXTemplate.KrakenX60(ELEVATOR_LEFT);
        rightMotor = RSTalonFXTemplate.KrakenX60(ELEVATOR_RIGHT);

        leftMotor.setNeutralMode(NeutralMode.BRAKE);
        rightMotor.setNeutralMode(NeutralMode.BRAKE);

        rightMotor.setControl(new Follower(ELEVATOR_LEFT, true));
    }

    @Override
    public void periodic() {
        super.periodic();
    }

    @Override
    public void initSendable(SendableBuilder builder) {
        builder.setSmartDashboardType("Elevator");
        builder.addDoubleProperty("ks", feedforward::getKs, feedforward::setKs);
        builder.addDoubleProperty("kg", feedforward::getKg, feedforward::setKg);
        builder.addDoubleProperty("ka", feedforward::getKa, feedforward::setKa);
        builder.addDoubleProperty("kv", feedforward::getKv, feedforward::setKv);
    }
}
