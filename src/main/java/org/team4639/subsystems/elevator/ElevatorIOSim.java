package org.team4639.subsystems.elevator;

import edu.wpi.first.wpilibj.smartdashboard.Mechanism2d;
import edu.wpi.first.wpilibj.smartdashboard.MechanismLigament2d;
import edu.wpi.first.wpilibj.smartdashboard.MechanismRoot2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import java.util.function.DoubleSupplier;

public class ElevatorIOSim implements ElevatorIO {
  private DoubleSupplier speedSupplier = () -> 0.0;
  private DoubleSupplier positionSupplier = () -> 0.0;

  private Mechanism2d elevatorView;
  private MechanismRoot2d elevatorViewRoot;
  private MechanismLigament2d elevatorLigament;

  public ElevatorIOSim() {
    elevatorView = new Mechanism2d(1, 3);
    elevatorViewRoot = elevatorView.getRoot("Elevator", 0.5, 0);
    elevatorLigament = elevatorViewRoot.append(new MechanismLigament2d("Elevator Ligament", 0, 0));
  }

  @Override
  public void updateInputs(ElevatorIOInputs inputs) {
    inputs.encoderMeasurement = positionSupplier.getAsDouble();
    inputs.encoderSpeed = speedSupplier.getAsDouble();

    elevatorLigament.setLength(
        ElevatorConstants.ProportionToPosition.convertBackwards(inputs.encoderMeasurement) * 3);
    SmartDashboard.putData("Elevator View", elevatorView);
  }

  @Override
  public void sendTalonInputs(DoubleSupplier speedSupplier, DoubleSupplier positionSupplier) {
    this.speedSupplier = speedSupplier;
    this.positionSupplier = positionSupplier;
  }
}
