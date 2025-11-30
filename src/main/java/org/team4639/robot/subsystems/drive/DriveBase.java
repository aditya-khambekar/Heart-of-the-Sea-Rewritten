package org.team4639.robot.subsystems.drive;

import com.ctre.phoenix6.CANBus;
import com.pathplanner.lib.config.ModuleConfig;
import com.pathplanner.lib.config.RobotConfig;
import edu.wpi.first.math.Matrix;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.math.numbers.N1;
import edu.wpi.first.math.numbers.N3;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine;
import java.util.Optional;
import java.util.concurrent.locks.Lock;
import java.util.concurrent.locks.ReentrantLock;
import org.littletonrobotics.junction.AutoLogOutput;
import org.team4639.lib.unit.Units2;
import org.team4639.robot.subsystems.drive.generated.TunerConstants;
import org.team4639.robot.subsystems.vision.Vision;

public abstract class DriveBase extends SubsystemBase implements Vision.VisionConsumer {

  @FunctionalInterface
  public interface DriveConsumer {
    void accept(
        Pose2d estimatedPose,
        SwerveModuleState[] moduleStates,
        SwerveModulePosition[] wheelPositions,
        Optional<Rotation2d> gyroAngle,
        double timestamp);
  }

  // PathPlanner config org.team4639.frobot.constants
  public static final double ROBOT_MASS_KG = Units2.poundsToKilograms.convert(136);
  public static final double ROBOT_MOI = 3.97;
  public static final double WHEEL_COF = 1.9;
  public static final RobotConfig PP_CONFIG =
      new RobotConfig(
          ROBOT_MASS_KG,
          ROBOT_MOI,
          new ModuleConfig(
              TunerConstants.FrontLeft.WheelRadius,
              11.0,
              // TunerConstants.kSpeedAt12Volts.in(MetersPerSecond),
              WHEEL_COF,
              DCMotor.getKrakenX60Foc(1)
                  .withReduction(TunerConstants.FrontLeft.DriveMotorGearRatio),
              TunerConstants.FrontLeft.SlipCurrent,
              1),
          getModuleTranslations());

  public DriveBase() {}

  // TunerConstants doesn't include these org.team4639.frobot.constants, so they are declared
  // locally
  public static final double ODOMETRY_FREQUENCY =
      new CANBus(TunerConstants.DrivetrainConstants.CANBusName).isNetworkFD() ? 250.0 : 100.0;
  public static final double DRIVE_BASE_RADIUS =
      Math.max(
          Math.max(
              Math.hypot(TunerConstants.FrontLeft.LocationX, TunerConstants.FrontLeft.LocationY),
              Math.hypot(TunerConstants.FrontRight.LocationX, TunerConstants.FrontRight.LocationY)),
          Math.max(
              Math.hypot(TunerConstants.BackLeft.LocationX, TunerConstants.BackLeft.LocationY),
              Math.hypot(TunerConstants.BackRight.LocationX, TunerConstants.BackRight.LocationY)));
  public static final Lock odometryLock = new ReentrantLock();
  public Pose2d setpoint = new Pose2d();

  /** Returns an array of module translations. */
  public static Translation2d[] getModuleTranslations() {
    return new Translation2d[] {
      new Translation2d(TunerConstants.FrontLeft.LocationX, TunerConstants.FrontLeft.LocationY),
      new Translation2d(TunerConstants.FrontRight.LocationX, TunerConstants.FrontRight.LocationY),
      new Translation2d(TunerConstants.BackLeft.LocationX, TunerConstants.BackLeft.LocationY),
      new Translation2d(TunerConstants.BackRight.LocationX, TunerConstants.BackRight.LocationY)
    };
  }

  public abstract boolean atSetpointTranslation();

  @Override
  public abstract void periodic();

  public abstract void runVelocity(ChassisSpeeds speeds);

  public abstract void runCharacterization(double output);

  public abstract void stop();

  public abstract void stopWithX();

  public abstract Command sysIdQuasistatic(SysIdRoutine.Direction direction);

  public abstract Command sysIdDynamic(SysIdRoutine.Direction direction);

  public abstract double[] getWheelRadiusCharacterizationPositions();

  public abstract double getFFCharacterizationVelocity();

  @AutoLogOutput(key = "Odometry/Robot")
  public abstract Pose2d getPose();

  public abstract Rotation2d getRotation();

  public abstract void setPose(Pose2d pose);

  public abstract void addVisionMeasurement(
      Pose2d visionRobotPoseMeters,
      double timestampSeconds,
      Matrix<N3, N1> visionMeasurementStdDevs);

  public abstract double getMaxLinearSpeedMetersPerSec();

  public abstract double getMaxAngularSpeedRadPerSec();

  @Override
  public abstract void accept(
      int cameraIndex,
      Pose2d visionRobotPoseMeters,
      double timestampSeconds,
      Matrix<N3, N1> visionMeasurementStdDevs);

  public abstract double[] getSpeeds();

  public abstract Field2d getField();
}
