package org.team4639.robot.commands;

import static edu.wpi.first.units.Units.Meters;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.util.sendable.Sendable;
import edu.wpi.first.util.sendable.SendableBuilder;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import org.team4639.lib.util.PoseUtil;
import org.team4639.robot.robot.Subsystems;

public class PIDToPoseCommand extends Command {
  private Pose2d setpoint;
  private PIDController xController;
  private PIDController yController;
  private PIDController headingController;

  public PIDToPoseCommand(Pose2d setpoint) {
    this.setpoint = setpoint;

    xController = new PIDController(0, 0, 0);
    yController = new PIDController(0, 0, 0);
    headingController = new PIDController(0, 0, 0);

    headingController.enableContinuousInput(-Math.PI, Math.PI);

    addRequirements(Subsystems.drive);

    SmartDashboard.putData("Training Align Heading Controller", headingController);
    SmartDashboard.putData(
        "Training Align XY Controllers",
        new Sendable() {

          @Override
          public void initSendable(SendableBuilder builder) {
            builder.addDoubleProperty(
                "kP",
                xController::getP,
                p -> {
                  xController.setP(p);
                  yController.setP(p);
                });
          }
        });
  }

  @Override
  public void initialize() {
    xController.setSetpoint(setpoint.getX());
    yController.setSetpoint(setpoint.getY());
    headingController.setSetpoint(setpoint.getRotation().getRadians());
  }

  @Override
  public void execute() {
    Subsystems.drive.runVelocity(
        ChassisSpeeds.fromFieldRelativeSpeeds(
            new ChassisSpeeds(
                xController.calculate(Subsystems.drive.getPose().getX()),
                yController.calculate(Subsystems.drive.getPose().getY()),
                headingController.calculate(Subsystems.drive.getPose().getRotation().getRadians())),
            Subsystems.drive.getPose().getRotation()));
  }

  @Override
  public void end(boolean interrupted) {
    Subsystems.drive.stopWithX();
  }

  @Override
  public boolean isFinished() {
    return PoseUtil.getDistance(setpoint, Subsystems.drive.getPose()).lte(Meters.of(0.1));
  }
}
