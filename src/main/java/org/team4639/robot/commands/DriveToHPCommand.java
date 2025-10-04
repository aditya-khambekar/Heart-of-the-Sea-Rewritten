package org.team4639.robot.commands;

import static org.team4639.robot.robot.Subsystems.drive;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.wpilibj2.command.Command;
import org.team4639.lib.unit.Units2;
import org.team4639.robot.robot.Subsystems;

public class DriveToHPCommand extends Command {
  private final Pose2d setpoint;
  private final ProfiledPIDController xController;
  private final PIDController thetaController;

  public DriveToHPCommand(Pose2d setpoint) {
    this.setpoint = setpoint;
    addRequirements(Subsystems.drive);
    xController = new ProfiledPIDController(3, 0, 0, new TrapezoidProfile.Constraints(8, 8));
    thetaController = new PIDController(24, 0, 0);
  }

  @Override
  public void initialize() {
    xController.reset(
        drivePoseRelativeToSetpoint().getX(), driveSpeedsRelativeToSetpoint().vxMetersPerSecond);
    xController.setGoal(0);
    xController.setTolerance(Units2.inchesToMeters.convert(0.5));
  }

  @Override
  public void execute() {
    drive.runVelocity(
        ChassisSpeeds.fromFieldRelativeSpeeds(
            new ChassisSpeeds(
                xController.getSetpoint().velocity
                    + xController.calculate(drivePoseRelativeToSetpoint().getX()),
                0,
                thetaController.calculate(
                    drivePoseRelativeToSetpoint().getRotation().getRadians())),
            drivePoseRelativeToSetpoint().getRotation()));
  }

  @Override
  public void end(boolean interrupted) {
    drive.stop();
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
}
