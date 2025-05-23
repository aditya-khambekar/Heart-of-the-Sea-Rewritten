package org.team4639._lib.control;

import static edu.wpi.first.units.Units.MetersPerSecond;

import com.pathplanner.lib.config.PIDConstants;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.units.measure.LinearVelocity;
import edu.wpi.first.util.sendable.Sendable;
import edu.wpi.first.util.sendable.SendableBuilder;
import edu.wpi.first.wpilibj.Timer;
import java.util.function.Supplier;

public class LERPProfiledPIDPoseController implements Sendable {
  ProfiledPIDPoseController controller;
  LinearVelocity LERPVelocity;
  Pose2d setpoint;
  Pose2d goal;
  double lastUpdateTime = 0.0;

  public LERPProfiledPIDPoseController(
      PIDConstants translationPIDGains,
      TrapezoidProfile.Constraints translationConstraints,
      PIDConstants rotationPIDGains,
      Supplier<ChassisSpeeds> drivetrainSpeedsRobotOriented,
      Supplier<Pose2d> drivetrainPose,
      LinearVelocity LERPVelocity) {
    controller =
        new ProfiledPIDPoseController(
            translationPIDGains,
            translationConstraints,
            rotationPIDGains,
            drivetrainSpeedsRobotOriented,
            drivetrainPose);
    this.LERPVelocity = LERPVelocity;
    this.lastUpdateTime = Timer.getFPGATimestamp();
  }

  @Override
  public void initSendable(SendableBuilder sendableBuilder) {
    controller.initSendable(sendableBuilder);
  }

  public void setGoal(Pose2d pose) {
    this.goal = pose;
  }

  public ChassisSpeeds calculateOutput(boolean fieldRelative) {
    double angleBetweenPoses =
        (Math.atan2((setpoint.getY() - goal.getY()), (setpoint.getX() - goal.getX())));
    double elapsedTime = Timer.getFPGATimestamp() - lastUpdateTime;
    setpoint =
        new Pose2d(
            setpoint.getX()
                + (LERPVelocity.in(MetersPerSecond) * Math.cos(angleBetweenPoses)) / elapsedTime,
            setpoint.getY()
                + (LERPVelocity.in(MetersPerSecond) * Math.sin(angleBetweenPoses)) / elapsedTime,
            goal.getRotation());
    controller.setGoal(setpoint);
    return controller.calculateOutput(fieldRelative);
  }
}
