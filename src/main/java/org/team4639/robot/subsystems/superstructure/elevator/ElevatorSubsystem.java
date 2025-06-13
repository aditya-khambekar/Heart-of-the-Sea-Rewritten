package org.team4639.robot.subsystems.superstructure.elevator;

import static edu.wpi.first.units.Units.Percent;

import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import org.team4639.robot.subsystems.superstructure.elevator.io.ElevatorIO;

public class ElevatorSubsystem extends SubsystemBase {
  private ElevatorIO io;
  private ElevatorIO.ElevatorIOInputs elevatorIOInputs;

  public ElevatorSubsystem(ElevatorIO io) {
    this.io = io;
    elevatorIOInputs = new ElevatorIO.ElevatorIOInputs();
    SmartDashboard.putData("Elevator", elevatorIOInputs);
  }

  public void elevatorUp() {
    io.setDutyCycleSetpoint(Percent.of(10));
  }

  public void elevatorDown() {
    io.setDutyCycleSetpoint(Percent.of(-10));
  }

  public void setMotionMagicSetpoint(Angle setpoint) {
    io.setMotionMagicSetpoint(setpoint);
  }

  @Override
  public void periodic() {
    io.updateInputs(elevatorIOInputs);
  }
}
