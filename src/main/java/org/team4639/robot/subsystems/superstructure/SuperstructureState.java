package org.team4639.robot.subsystems.superstructure;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.units.measure.Dimensionless;

public record SuperstructureState(
    Dimensionless elevatorProportion, Rotation2d wristRotation, AngularVelocity wheelSpeed) {}
