// Copyright 2021-2025 FRC 6328
// http://github.com/Mechanical-Advantage
//
// This program is free software; you can redistribute it and/or
// modify it under the terms of the GNU General Public License
// version 3 as published by the Free Software Foundation or
// available in the root directory of this project.
//
// This program is distributed in the hope that it will be useful,
// but WITHOUT ANY WARRANTY; without even the implied warranty of
// MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the
// GNU General Public License for more details.

package org.team4639.robot.commands;

import static org.team4639.robot.robot.Subsystems.drive;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.Vector;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.filter.SlewRateLimiter;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Transform2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.numbers.N2;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import java.text.DecimalFormat;
import java.text.NumberFormat;
import java.util.LinkedList;
import java.util.List;
import java.util.function.DoubleSupplier;
import java.util.function.Supplier;
import org.team4639.lib.util.PoseUtil;
import org.team4639.robot.constants.FieldConstants;
import org.team4639.robot.robot.Subsystems;
import org.team4639.robot.subsystems.drive.Drive;

public class DriveCommands {
  private static final double DEADBAND = 0.1;
  private static final double ANGLE_KP = 32.0;
  private static final double ANGLE_KD = 0.4;
  private static final double ANGLE_KI = 0.01;
  private static final double ANGLE_MAX_VELOCITY = 15.0;
  private static final double ANGLE_MAX_ACCELERATION = 20.0;
  private static final double FF_START_DELAY = 2.0; // Secs
  private static final double FF_RAMP_RATE = 0.1; // Volts/Sec
  private static final double WHEEL_RADIUS_MAX_VELOCITY = 0.25; // Rad/Sec
  private static final double WHEEL_RADIUS_RAMP_RATE = 0.05; // Rad/Sec^2
  private static final double ALIGN_FF = 1.0;

  private DriveCommands() {}

  private static Translation2d getLinearVelocityFromJoysticks(double x, double y) {
    // Apply deadband
    double linearMagnitude = MathUtil.applyDeadband(Math.hypot(x, y), DEADBAND);
    Rotation2d linearDirection = new Rotation2d(Math.atan2(y, x));

    // Square magnitude for more precise control
    linearMagnitude = 3 * linearMagnitude * linearMagnitude;

    // Return new linear velocity
    return new Pose2d(new Translation2d(), linearDirection)
        .transformBy(new Transform2d(linearMagnitude, 0.0, new Rotation2d()))
        .getTranslation();
  }

  /**
   * Field relative drive command using two joysticks (controlling linear and angular velocities).
   */
  public static Command joystickDrive(
      DoubleSupplier xSupplier, DoubleSupplier ySupplier, DoubleSupplier omegaSupplier) {
    return Commands.run(
        () -> {
          // Get linear velocity
          Translation2d linearVelocity =
              getLinearVelocityFromJoysticks(xSupplier.getAsDouble(), ySupplier.getAsDouble());

          // Apply rotation deadband
          double omega = MathUtil.applyDeadband(omegaSupplier.getAsDouble(), DEADBAND);

          // Square rotation value for more precise control
          omega = Math.copySign(omega * omega, omega);

          // Convert to field relative speeds & send command
          ChassisSpeeds speeds =
              new ChassisSpeeds(
                  linearVelocity.getX() * drive.getMaxLinearSpeedMetersPerSec(),
                  linearVelocity.getY() * drive.getMaxLinearSpeedMetersPerSec(),
                  omega * drive.getMaxAngularSpeedRadPerSec());
          boolean isFlipped = false;
          drive.runVelocity(ChassisSpeeds.fromFieldRelativeSpeeds(speeds, drive.getRotation()));

          SmartDashboard.putNumber(
              "Velocity", Math.hypot(speeds.vxMetersPerSecond, speeds.vyMetersPerSecond));
        },
        drive);
  }

  /**
   * Field relative drive command using joystick for linear control and PID for angular control.
   * Possible use cases include snapping to an angle, aiming at a vision target, or controlling
   * absolute rotation with a joystick.
   */
  public static Command joystickDriveAtAngle(
      DoubleSupplier xSupplier, DoubleSupplier ySupplier, Supplier<Rotation2d> rotationSupplier) {

    // Create PID controller
    ProfiledPIDController angleController =
        new ProfiledPIDController(
            ANGLE_KP,
            ANGLE_KI,
            ANGLE_KD,
            new TrapezoidProfile.Constraints(ANGLE_MAX_VELOCITY, ANGLE_MAX_ACCELERATION));
    angleController.enableContinuousInput(-Math.PI, Math.PI);

    // Construct command
    return Commands.run(
            () -> {
              // Get linear velocity
              Translation2d linearVelocity =
                  getLinearVelocityFromJoysticks(xSupplier.getAsDouble(), ySupplier.getAsDouble());

              // Calculate angular speed
              double omega =
                  angleController.calculate(
                      drive.getRotation().getRadians(), rotationSupplier.get().getRadians());

              // Convert to field relative speeds & send command
              ChassisSpeeds speeds =
                  new ChassisSpeeds(
                      linearVelocity.getX() * drive.getMaxLinearSpeedMetersPerSec(),
                      linearVelocity.getY() * drive.getMaxLinearSpeedMetersPerSec(),
                      omega);
              boolean isFlipped = false;
              drive.runVelocity(
                  ChassisSpeeds.fromFieldRelativeSpeeds(
                      speeds,
                      isFlipped
                          ? drive.getRotation().plus(new Rotation2d(Math.PI))
                          : drive.getRotation()));

              SmartDashboard.putNumber(
                  "Velocity", Math.hypot(speeds.vxMetersPerSecond, speeds.vyMetersPerSecond));
            },
            drive)

        // Reset PID controller when command starts
        .beforeStarting(() -> angleController.reset(drive.getRotation().getRadians()));
  }

  /**
   * Field relative drive command using joystick for linear control and PID for angular control,
   * also includes a translation vector that allows for dynamic combination of alignment and driver
   * input. Possible use cases include snapping to an angle, aiming at a vision target, or
   * controlling absolute rotation with a joystick.
   */
  public static Command joystickDriveAtAngleWithTranslationVector(
      DoubleSupplier xSupplier,
      DoubleSupplier ySupplier,
      Supplier<Rotation2d> rotationSupplier,
      Supplier<Vector<N2>> vectorSupplier) {

    // Create PID controller
    ProfiledPIDController angleController =
        new ProfiledPIDController(
            ANGLE_KP,
            ANGLE_KI,
            ANGLE_KD,
            new TrapezoidProfile.Constraints(ANGLE_MAX_VELOCITY, ANGLE_MAX_ACCELERATION));
    angleController.enableContinuousInput(-Math.PI, Math.PI);

    // Construct command
    return Commands.run(
            () -> {
              Vector<N2> joystickInput =
                  VecBuilder.fill(xSupplier.getAsDouble(), ySupplier.getAsDouble());
              Vector<N2> finalInput = joystickInput.plus(vectorSupplier.get());
              // Get linear velocity
              Translation2d linearVelocity =
                  getLinearVelocityFromJoysticks(finalInput.get(0), finalInput.get(1));

              // Calculate angular speed
              double omega =
                  angleController.calculate(
                      drive.getRotation().getRadians(), rotationSupplier.get().getRadians());

              // Convert to field relative speeds & send command
              ChassisSpeeds speeds =
                  new ChassisSpeeds(
                      linearVelocity.getX() * drive.getMaxLinearSpeedMetersPerSec(),
                      linearVelocity.getY() * drive.getMaxLinearSpeedMetersPerSec(),
                      omega);
              boolean isFlipped = false;
              drive.runVelocity(
                  ChassisSpeeds.fromFieldRelativeSpeeds(
                      speeds,
                      isFlipped
                          ? drive.getRotation().plus(new Rotation2d(Math.PI))
                          : drive.getRotation()));

              SmartDashboard.putNumber(
                  "Velocity", Math.hypot(speeds.vxMetersPerSecond, speeds.vyMetersPerSecond));
            },
            drive)

        // Reset PID controller when command starts
        .beforeStarting(() -> angleController.reset(drive.getRotation().getRadians()));
  }

  /**
   * Measures the velocity feedforward org.team4639.frobot.constants for the drive motors.
   *
   * <p>This command should only be used in voltage control mode.
   */
  public static Command feedforwardCharacterization(Drive drive) {
    List<Double> velocitySamples = new LinkedList<>();
    List<Double> voltageSamples = new LinkedList<>();
    Timer timer = new Timer();

    return Commands.sequence(
        // Reset data
        Commands.runOnce(
            () -> {
              velocitySamples.clear();
              voltageSamples.clear();
            }),

        // Allow modules to orient
        Commands.run(
                () -> {
                  drive.runCharacterization(0.0);
                },
                drive)
            .withTimeout(FF_START_DELAY),

        // Start timer
        Commands.runOnce(timer::restart),

        // Accelerate and gather data
        Commands.run(
                () -> {
                  double voltage = timer.get() * FF_RAMP_RATE;
                  drive.runCharacterization(voltage);
                  velocitySamples.add(drive.getFFCharacterizationVelocity());
                  voltageSamples.add(voltage);
                },
                drive)

            // When cancelled, calculate and print results
            .finallyDo(
                () -> {
                  int n = velocitySamples.size();
                  double sumX = 0.0;
                  double sumY = 0.0;
                  double sumXY = 0.0;
                  double sumX2 = 0.0;
                  for (int i = 0; i < n; i++) {
                    sumX += velocitySamples.get(i);
                    sumY += voltageSamples.get(i);
                    sumXY += velocitySamples.get(i) * voltageSamples.get(i);
                    sumX2 += velocitySamples.get(i) * velocitySamples.get(i);
                  }
                  double kS = (sumY * sumX2 - sumX * sumXY) / (n * sumX2 - sumX * sumX);
                  double kV = (n * sumXY - sumX * sumY) / (n * sumX2 - sumX * sumX);

                  NumberFormat formatter = new DecimalFormat("#0.00000");
                  System.out.println("********** Drive FF Characterization Results **********");
                  System.out.println("\tkS: " + formatter.format(kS));
                  System.out.println("\tkV: " + formatter.format(kV));
                }));
  }

  /** Measures the robot's wheel radius by spinning in a circle. */
  public static Command wheelRadiusCharacterization(Drive drive) {
    SlewRateLimiter limiter = new SlewRateLimiter(WHEEL_RADIUS_RAMP_RATE);
    WheelRadiusCharacterizationState state = new WheelRadiusCharacterizationState();

    return Commands.parallel(
        // Drive control sequence
        Commands.sequence(
            // Reset acceleration limiter
            Commands.runOnce(
                () -> {
                  limiter.reset(0.0);
                }),

            // Turn in place, accelerating up to full speed
            Commands.run(
                () -> {
                  double speed = limiter.calculate(WHEEL_RADIUS_MAX_VELOCITY);
                  drive.runVelocity(new ChassisSpeeds(0.0, 0.0, speed));
                },
                drive)),

        // Measurement sequence
        Commands.sequence(
            // Wait for modules to fully orient before starting measurement
            Commands.waitSeconds(1.0),

            // Record starting measurement
            Commands.runOnce(
                () -> {
                  state.positions = drive.getWheelRadiusCharacterizationPositions();
                  state.lastAngle = drive.getRotation();
                  state.gyroDelta = 0.0;
                }),

            // Update gyro delta
            Commands.run(
                    () -> {
                      var rotation = drive.getRotation();
                      state.gyroDelta += Math.abs(rotation.minus(state.lastAngle).getRadians());
                      state.lastAngle = rotation;
                    })

                // When cancelled, calculate and print results
                .finallyDo(
                    () -> {
                      double[] positions = drive.getWheelRadiusCharacterizationPositions();
                      double wheelDelta = 0.0;
                      for (int i = 0; i < 4; i++) {
                        wheelDelta += Math.abs(positions[i] - state.positions[i]) / 4.0;
                      }
                      double wheelRadius = (state.gyroDelta * Drive.DRIVE_BASE_RADIUS) / wheelDelta;

                      NumberFormat formatter = new DecimalFormat("#0.000");
                      System.out.println(
                          "********** Wheel Radius Characterization Results **********");
                      System.out.println(
                          "\tWheel Delta: " + formatter.format(wheelDelta) + " radians");
                      System.out.println(
                          "\tGyro Delta: " + formatter.format(state.gyroDelta) + " radians");
                      System.out.println(
                          "\tWheel Radius: "
                              + formatter.format(wheelRadius)
                              + " meters, "
                              + formatter.format(Units.metersToInches(wheelRadius))
                              + " inches");
                    })));
  }

  private static class WheelRadiusCharacterizationState {
    double[] positions = new double[4];
    Rotation2d lastAngle = new Rotation2d();
    double gyroDelta = 0.0;
  }

  public static Command alignToNearestReef() {
    Pose2d drivePose = drive.getPose();

    Pose2d nearestReefPose =
        drivePose.nearest(
            List.of(
                FieldConstants.TargetPositions.REEF_AB.getPose(),
                FieldConstants.TargetPositions.REEF_CD.getPose(),
                FieldConstants.TargetPositions.REEF_EF.getPose(),
                FieldConstants.TargetPositions.REEF_GH.getPose(),
                FieldConstants.TargetPositions.REEF_IJ.getPose(),
                FieldConstants.TargetPositions.REEF_KL.getPose()));

    return PIDToPose(nearestReefPose);
  }

  public static Command alignToNearestReefLeft() {
    Pose2d drivePose = drive.getPose();

    Pose2d nearestReefPose =
        PoseUtil.ReefRelativeLeftOf(
            drivePose.nearest(
                List.of(
                    FieldConstants.TargetPositions.REEF_AB.getPose(),
                    FieldConstants.TargetPositions.REEF_CD.getPose(),
                    FieldConstants.TargetPositions.REEF_EF.getPose(),
                    FieldConstants.TargetPositions.REEF_GH.getPose(),
                    FieldConstants.TargetPositions.REEF_IJ.getPose(),
                    FieldConstants.TargetPositions.REEF_KL.getPose())));

    return PIDToReefPose(nearestReefPose);
  }

  public static Command alignToNearestReefRight() {
    Pose2d drivePose = drive.getPose();

    Pose2d nearestReefPose =
        PoseUtil.ReefRelativeRightOf(
            drivePose.nearest(
                List.of(
                    FieldConstants.TargetPositions.REEF_AB.getPose(),
                    FieldConstants.TargetPositions.REEF_CD.getPose(),
                    FieldConstants.TargetPositions.REEF_EF.getPose(),
                    FieldConstants.TargetPositions.REEF_GH.getPose(),
                    FieldConstants.TargetPositions.REEF_IJ.getPose(),
                    FieldConstants.TargetPositions.REEF_KL.getPose())));

    return PIDToReefPose(nearestReefPose);
  }

  public static Command PIDToPose(Pose2d destinationPose) {
    ProfiledPIDController pidX =
        new ProfiledPIDController(3, 0, 0, new TrapezoidProfile.Constraints(3, 5));
    ProfiledPIDController pidY =
        new ProfiledPIDController(3, 0, 0, new TrapezoidProfile.Constraints(3, 5));
    PIDController headingController = new PIDController(16, 0, 0);
    headingController.enableContinuousInput(-Math.PI, Math.PI);
    headingController.setSetpoint(destinationPose.getRotation().getRadians());

    var startingPose = drive.getPose();

    pidX.reset(startingPose.getX(), drive.getSpeeds()[0]);
    pidX.setGoal(destinationPose.getX());

    pidY.reset(startingPose.getY(), drive.getSpeeds()[1]);
    pidY.setGoal(destinationPose.getY());

    drive.setpoint = destinationPose;

    double kp = 6;

    return drive
        .run(
            () -> {
              drive.runVelocity(
                  ChassisSpeeds.fromFieldRelativeSpeeds(
                      new ChassisSpeeds(
                          ALIGN_FF * pidX.getSetpoint().velocity
                              + pidX.calculate(drive.getPose().getX()),
                          ALIGN_FF * pidY.getSetpoint().velocity
                              + pidY.calculate(drive.getPose().getY()),
                          headingController.calculate(drive.getPose().getRotation().getRadians())),
                      drive.getPose().getRotation()));
              drive
                  .getField()
                  .getObject("PID Setpoint")
                  .setPose(
                      new Pose2d(
                          pidX.getSetpoint().position,
                          pidY.getSetpoint().position,
                          Rotation2d.fromRadians(headingController.getSetpoint())));
            })
        .until(drive::atSetpointTranslation);
  }

  public static Command PIDToReefPose(Pose2d destinationPose) {
    ProfiledPIDController pidX =
        new ProfiledPIDController(3, 0, 0, new TrapezoidProfile.Constraints(8, 8));
    ProfiledPIDController pidY =
        new ProfiledPIDController(3, 0, 0, new TrapezoidProfile.Constraints(8, 8));
    PIDController headingController = new PIDController(32, 0, 0);
    headingController.enableContinuousInput(-Math.PI, Math.PI);
    headingController.setSetpoint(destinationPose.getRotation().getRadians());

    var startingPose = drive.getPose();

    pidX.reset(startingPose.getX(), drive.getSpeeds()[0]);
    pidX.setGoal(destinationPose.getX());

    pidY.reset(startingPose.getY(), drive.getSpeeds()[1]);
    pidY.setGoal(destinationPose.getY());

    drive.setpoint = destinationPose;

    double kp = 6;

    return drive
        .run(
            () -> {
              headingController.setSetpoint(
                  FieldConstants.getRotationToClosestBranchPosition(drive.getPose()).getRadians());
              drive.runVelocity(
                  ChassisSpeeds.fromFieldRelativeSpeeds(
                      new ChassisSpeeds(
                          ALIGN_FF * pidX.getSetpoint().velocity
                              + pidX.calculate(drive.getPose().getX()),
                          ALIGN_FF * pidY.getSetpoint().velocity
                              + pidY.calculate(drive.getPose().getY()),
                          headingController.calculate(drive.getPose().getRotation().getRadians())),
                      drive.getPose().getRotation()));
              drive
                  .getField()
                  .getObject("PID Setpoint")
                  .setPose(
                      new Pose2d(
                          pidX.getSetpoint().position,
                          pidY.getSetpoint().position,
                          Rotation2d.fromRadians(headingController.getSetpoint())));
            })
        .until(drive::atSetpointTranslation);
  }

  public static Command robotOrientedDrive(Drive drive, ChassisSpeeds speeds) {
    return drive.run(() -> drive.runVelocity(speeds));
  }

  public static Command HPLeftAlign(Drive drive) {
    return Subsystems.drive.defer(
        () -> PIDToPose(FieldConstants.TargetPositions.HP_LEFT.getPose()));
  }

  public static Command HPRightAlign(Drive drive) {
    return Subsystems.drive.defer(
        () -> PIDToPose(FieldConstants.TargetPositions.HP_RIGHT.getPose()));
  }

  public static Command stopWithX() {
    return Subsystems.drive.run(() -> Subsystems.drive.stopWithX()).withName("STOP_WITH_X");
  }
}
