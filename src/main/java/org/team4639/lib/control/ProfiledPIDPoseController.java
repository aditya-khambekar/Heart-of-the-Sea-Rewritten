package org.team4639.lib.control;

import com.pathplanner.lib.config.PIDConstants;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.util.sendable.Sendable;
import edu.wpi.first.util.sendable.SendableBuilder;
import java.util.function.Supplier;
import org.team4639.lib.util.ArraysUtil;

public class ProfiledPIDPoseController implements Sendable {
  private ProfiledPIDController xController;
  private ProfiledPIDController yController;
  private PIDController headingController;

  private Supplier<ChassisSpeeds> drivetrainSpeedsFieldOriented;
  private Supplier<Pose2d> drivetrainPose;

  private Pose2d goal;

  public ProfiledPIDPoseController(
      PIDConstants translationPIDGains,
      TrapezoidProfile.Constraints translationConstraints,
      PIDConstants rotationPIDGains,
      Supplier<ChassisSpeeds> drivetrainSpeedsRobotOriented,
      Supplier<Pose2d> drivetrainPose) {
    xController =
        new ProfiledPIDController(
            translationPIDGains.kP,
            translationPIDGains.kI,
            translationPIDGains.kD,
            translationConstraints);
    yController =
        new ProfiledPIDController(
            translationPIDGains.kP,
            translationPIDGains.kI,
            translationPIDGains.kD,
            translationConstraints);

    headingController =
        new PIDController(rotationPIDGains.kP, rotationPIDGains.kI, rotationPIDGains.kD);

    drivetrainSpeedsFieldOriented =
        () ->
            ChassisSpeeds.fromRobotRelativeSpeeds(
                drivetrainSpeedsRobotOriented.get(), drivetrainPose.get().getRotation());
    this.drivetrainPose = drivetrainPose;
    xController.reset(
        drivetrainPose.get().getX(), drivetrainSpeedsFieldOriented.get().vxMetersPerSecond);
    yController.reset(
        drivetrainPose.get().getY(), drivetrainSpeedsFieldOriented.get().vyMetersPerSecond);
  }

  @Override
  public void initSendable(SendableBuilder builder) {
    builder.addDoubleArrayProperty(
        "Translation PIDs",
        () ->
            ArraysUtil.createDoubleArray(
                xController.getP(), xController.getI(), xController.getD()),
        x -> {
          xController.setPID(x[0], x[1], x[2]);
          yController.setPID(x[0], x[1], x[2]);
        });

    builder.addDoubleArrayProperty(
        "Translation Constraints",
        () ->
            ArraysUtil.createDoubleArray(
                xController.getConstraints().maxVelocity,
                xController.getConstraints().maxAcceleration),
        x -> {
          xController.setConstraints(new TrapezoidProfile.Constraints(x[0], x[1]));
          yController.setConstraints(new TrapezoidProfile.Constraints(x[0], x[1]));
        });

    builder.addDoubleArrayProperty(
        "Heading PIDs",
        () ->
            ArraysUtil.createDoubleArray(
                headingController.getP(), headingController.getI(), headingController.getD()),
        x -> {
          headingController.setPID(x[0], x[1], x[2]);
        });
  }

  public void setGoal(Pose2d pose) {
    this.goal = pose;
    xController.setGoal(pose.getX());
    yController.setGoal(pose.getY());
    headingController.setSetpoint(pose.getRotation().getRadians());
  }

  public ChassisSpeeds calculateOutput(boolean fieldOriented) {
    return ChassisSpeeds.fromFieldRelativeSpeeds(
        xController.getSetpoint().velocity + xController.calculate(drivetrainPose.get().getX()),
        yController.getSetpoint().velocity + yController.calculate(drivetrainPose.get().getY()),
        headingController.calculate(drivetrainPose.get().getRotation().getRadians()),
        drivetrainPose.get().getRotation());
  }

  public Pose2d getGoal() {
    return goal;
  }

  public Pose2d getSetpoint() {
    return new Pose2d(
        xController.getSetpoint().position,
        yController.getSetpoint().position,
        Rotation2d.fromRadians(headingController.getSetpoint()));
  }
}
