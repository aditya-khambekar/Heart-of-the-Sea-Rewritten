package org.team4639.robot.subsystems.superstructure.wrist.io;

import edu.wpi.first.units.Units;
import edu.wpi.first.units.measure.*;
import edu.wpi.first.util.sendable.Sendable;
import edu.wpi.first.util.sendable.SendableBuilder;

import java.util.Optional;

import static edu.wpi.first.units.Units.*;

public abstract class WristIO {
    public static class WristIOInputs implements Sendable {
        Angle motorPosition;
        AngularVelocity motorVelocity;
        Temperature motorTemperature;
        Current motorCurrent;

        @Override
        public void initSendable(SendableBuilder builder) {
            builder.addDoubleProperty(
                    "Motor Rotations",
                    () -> Optional.ofNullable(motorPosition).orElse(Rotations.zero()).in(Rotations),
                    null);
            builder.addDoubleProperty(
                    "Motor Speed Rad/s",
                    () ->
                            Optional.ofNullable(motorVelocity)
                                    .orElse(RadiansPerSecond.zero())
                                    .in(RadiansPerSecond),
                    null);
            builder.addDoubleProperty(
                    "Motor Celsius",
                    () -> Optional.ofNullable(motorTemperature).orElse(Celsius.zero()).in(Celsius),
                    null);
            builder.addDoubleProperty(
                    "Motor Current",
                    () -> Optional.ofNullable(motorCurrent).orElse(Amps.zero()).in(Units.Amp),
                    null);
        }
    }

    public abstract void updateInputs(WristIOInputs inputs);

    public abstract void setDutyCycleOutput(Dimensionless percent);

    public abstract void setPosition(Angle position);
}
