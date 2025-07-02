package org.team4639.robot.commands.superstructure;

import static edu.wpi.first.units.Units.RotationsPerSecond;
import static edu.wpi.first.units.Units.Value;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.units.measure.MutDimensionless;
import org.team4639.robot.constants.Controls;
import org.team4639.robot.robot.Subsystems;
import org.team4639.robot.subsystems.superstructure.Superstructure;

public class MicroAdjustmentCommand extends SuperstructureCommandBase {
  private MutDimensionless elevatorSetpoint;
  private Rotation2d wristSetpoint;

  public MicroAdjustmentCommand() {
    addRequirements(
        Subsystems.superstructure, Subsystems.elevator, Subsystems.wrist, Subsystems.roller);
  }

  @Override
  public void initialize() {
    elevatorSetpoint = Superstructure.getSuperstructureState().elevatorProportion().mutableCopy();
    wristSetpoint =
        Rotation2d.fromDegrees(
            Superstructure.getSuperstructureState().wristRotation().getDegrees());
  }

  public void execute() {
    super.execute();
    if (Controls.elevatorUp.getAsBoolean()) elevatorSetpoint.mut_plus(Value.of(0.001));
    else if (Controls.elevatorDown.getAsBoolean()) elevatorSetpoint.mut_minus(Value.of(0.001));

    if (Controls.wristUp.getAsBoolean())
      wristSetpoint = wristSetpoint.plus(Rotation2d.fromDegrees(0.1));
    else if (Controls.wristDown.getAsBoolean())
      wristSetpoint = wristSetpoint.minus(Rotation2d.fromDegrees(0.1));

    Subsystems.elevator.setPercentageRaw(elevatorSetpoint);
    Subsystems.wrist.setWristSetpoint(wristSetpoint);

    // UNTUNED
    if (Controls.outtake.getAsBoolean()) Subsystems.roller.setVelocity(RotationsPerSecond.of(15));
  }

  @Override
  public SuperstructureCommandState getState() {
    return SuperstructureCommandState.MICROADJUSTMENTS;
  }

  @Override
  public String getName() {
    return "MICROADJUSTMENTS";
  }
}
