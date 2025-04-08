package org.team4639.subsystems.elevator;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import java.util.function.DoubleSupplier;

import org.team4639._lib.motorcontrol.talonfx.RSTalonFX;
import org.team4639._robot.Subsystems;

import com.ctre.phoenix6.hardware.TalonFX;

public class ElevatorIOSim implements ElevatorIO {
  private DoubleSupplier speedSupplier = () -> 0.0;
  private DoubleSupplier positionSupplier = () -> 0.0;



  @Override
  public void updateInputs(ElevatorIOInputs inputs) {
    inputs.encoderMeasurement = positionSupplier.getAsDouble();
    inputs.encoderSpeed = speedSupplier.getAsDouble();
  }

  @Override
  public void sendTalonInputs(TalonFX talon) {
    this.speedSupplier = () -> talon.getVelocity().getValueAsDouble();
    this.positionSupplier = () -> talon.getPosition().getValueAsDouble();
  }
}
