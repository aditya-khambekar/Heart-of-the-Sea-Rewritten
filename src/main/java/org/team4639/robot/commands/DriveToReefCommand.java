package org.team4639.robot.commands;

import static org.team4639.robot.robot.Subsystems.drive;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.filter.SlewRateLimiter;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import lombok.Getter;
import org.team4639.lib.tunable.TunableNumber;
import org.team4639.lib.unit.Units2;
import org.team4639.robot.robot.Subsystems;

public class DriveToReefCommand extends Command {

  private final Pose2d setpoint;
  @Getter private DTRCState state;
  private final ProfiledPIDController xController;
  private final ProfiledPIDController yController;
  private final PIDController thetaController;

  private final TunableNumber cosPower =
      new TunableNumber().withDefaultValue(1001).send("New Align/Cos Power");
  private final TunableNumber inXVelocity =
      new TunableNumber().withDefaultValue(-1).send("New Align/In X Velocity");
  private final TunableNumber whileScoringXSpeed =
      new TunableNumber().withDefaultValue(-0.5).send("New Align/While Scoring Speed");
  private final TunableNumber outXVelocity =
      new TunableNumber().withDefaultValue(-1).send("New Align/Out X Velocity");
  private final SlewRateLimiter xLimiter = new SlewRateLimiter(5);
  private final SlewRateLimiter yLimiter = new SlewRateLimiter(5);

  public DriveToReefCommand(Pose2d setpoint) {
    this.setpoint = setpoint;
    addRequirements(Subsystems.drive);

    xController = new ProfiledPIDController(3, 0, 0, new TrapezoidProfile.Constraints(1, 8));
    yController = new ProfiledPIDController(5, 0, 0, new TrapezoidProfile.Constraints(12, 8));
    thetaController = new PIDController(24, 0, 0);
  }

  @Override
  public void initialize() {
    this.state = DTRCState.ALIGN;
    xController.reset(
        drivePoseRelativeToSetpoint().getX(), driveSpeedsRelativeToSetpoint().vxMetersPerSecond);
    xController.setGoal(0);
    xController.setTolerance(Units2.inchesToMeters.convert(0.5));
    yController.reset(
        drivePoseRelativeToSetpoint().getY(), driveSpeedsRelativeToSetpoint().vyMetersPerSecond);
    yController.setGoal(0);
    thetaController.reset();
    thetaController.setSetpoint(0);

    Subsystems.drive.setpoint = setpoint;

    SmartDashboard.putData("X Controller", xController);
  }

  @Override
  public void execute() {
    switch (state) {
      case ALIGN:
        xController.calculate(drivePoseRelativeToSetpoint().getX());
        drive.runVelocity(
            ChassisSpeeds.fromFieldRelativeSpeeds(
                new ChassisSpeeds(
                    xController.getSetpoint().velocity
                        + Math.pow(getRotationToSetpoint().getCos(), cosPower.get())
                            * inXVelocity.get(),
                    yController.getSetpoint().velocity
                        + yController.calculate(drivePoseRelativeToSetpoint().getY()),
                    thetaController.calculate(
                        drivePoseRelativeToSetpoint().getRotation().getRadians())),
                drivePoseRelativeToSetpoint().getRotation()));
        if (drivePoseRelativeToSetpoint().getX() >= 0) {
          setState(DTRCState.SCORE);
        }
        break;
      case SCORE:
        drive.runVelocity(
            new ChassisSpeeds(
                xLimiter.calculate(whileScoringXSpeed.getAsDouble()), yLimiter.calculate(0), 0));
        if (Subsystems.wrist.doesNotHaveCoral()) setState(DTRCState.OUT);
        break;
      case OUT:
        drive.runVelocity(
            new ChassisSpeeds(
                xLimiter.calculate(outXVelocity.getAsDouble()), yLimiter.calculate(0), 0));
        break;
    }

    xLimiter.calculate(driveSpeedsRelativeToSetpoint().vxMetersPerSecond);
    yLimiter.calculate(driveSpeedsRelativeToSetpoint().vyMetersPerSecond);

    SmartDashboard.putString("Align Command State", getState().name());
  }

  public static enum DTRCState {
    ALIGN,
    SCORE,
    OUT;
  }

  private Pose2d drivePoseRelativeToSetpoint() {
    return Subsystems.drive.getPose().relativeTo(setpoint);
  }

  private ChassisSpeeds driveSpeedsRelativeToSetpoint() {
    return ChassisSpeeds.fromRobotRelativeSpeeds(
        Subsystems.drive.getChassisSpeeds(), drivePoseRelativeToSetpoint().getRotation());
  }

  public static Rotation2d getRotationToSetpoint() {
    return Rotation2d.fromRadians(Math.atan2(-drive.getPose().getY(), -drive.getPose().getX()));
  }

  private void setState(DTRCState state) {
    this.state = state;
  }

  @Override
  public boolean isFinished() {
    return (state == DTRCState.OUT);
  }
}
