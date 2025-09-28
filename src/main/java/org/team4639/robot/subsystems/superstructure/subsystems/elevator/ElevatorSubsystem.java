package org.team4639.robot.subsystems.superstructure.subsystems.elevator;

import static edu.wpi.first.units.Units.Percent;
import static edu.wpi.first.units.Units.Rotations;
import static edu.wpi.first.units.Units.RotationsPerSecond;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.filter.Debouncer;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.units.measure.Dimensionless;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import java.util.Optional;
import lombok.Getter;
import org.team4639.robot.robot.Subsystems;
import org.team4639.robot.subsystems.superstructure.subsystems.elevator.io.ElevatorIO;

public class ElevatorSubsystem extends SubsystemBase {
  @Getter private final ElevatorIO io;
  @Getter private final ElevatorIO.ElevatorIOInputs inputs;
  private final Debouncer stoppedDebouncer = new Debouncer(0.5, Debouncer.DebounceType.kRising);

  public ElevatorSubsystem(ElevatorIO io) {
    this.io = io;
    inputs = new ElevatorIO.ElevatorIOInputs();
    SmartDashboard.putData("Elevator", inputs);
  }

  public void elevatorUp() {
    io.setDutyCycleSetpoint(Percent.of(0.1));
  }

  public void elevatorDown() {
    io.setDutyCycleSetpoint(Percent.of(-0.2));
  }

  public void elevatorStop() {
    io.setDutyCycleSetpoint(Percent.of(0));
  }

  private void setMotionMagicSetpoint(Angle setpoint) {
    io.setMotionMagicSetpoint(setpoint);
    SmartDashboard.putNumber("Setpoint Rotations", setpoint.in(Rotations));
  }

  public void setPercentageRaw(Dimensionless percentage) {
    setMotionMagicSetpoint(ElevatorConstants.rotationsToPercentage.convertBackwards(percentage));
    SmartDashboard.putNumber(
        "Motion Magic Setpoint",
        ElevatorConstants.rotationsToPercentage.convertBackwards(percentage).in(Rotations));
  }

  @Override
  public void periodic() {
    io.updateInputs(inputs);
    SmartDashboard.putString(
        "Elevator Command",
        Optional.ofNullable(this.getCurrentCommand()).map(x -> x.getName()).orElse("NONE"));
  }

  public void elevatorHoldRaw() {
    setMotionMagicSetpoint(inputs.rightMotorPosition.copy());
  }

  public Dimensionless getPercentage() {
    return ElevatorConstants.rotationsToPercentage.convert(inputs.rightMotorPosition);
  }

  public void zero() {
    io.zero();
  }

  public AngularVelocity getMotorSpeed() {
    return inputs.leftMotorSpeed;
  }

  public boolean isElevatorPhysicallyStopped() {
    return stoppedDebouncer.calculate(
        MathUtil.isNear(0, Subsystems.elevator.getMotorSpeed().in(RotationsPerSecond), 0.1));
  }
}
