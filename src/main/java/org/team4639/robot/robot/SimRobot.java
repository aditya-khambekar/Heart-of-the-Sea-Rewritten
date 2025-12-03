package org.team4639.robot.robot;

import static edu.wpi.first.units.Units.Meters;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.networktables.StructArrayPublisher;
import org.ironmaple.simulation.drivesims.SwerveDriveSimulation;
import org.littletonrobotics.junction.Logger;
import org.team4639.robot.subsystems.drive.maple.MDrive;
import org.team4639.robot.subsystems.superstructure.subsystems.elevator.ElevatorConstants;
import org.team4639.robot.subsystems.superstructure.subsystems.wrist.WristConstants;

public class SimRobot {
  public static SwerveDriveSimulation driveSimulation =
      new SwerveDriveSimulation(MDrive.mapleSimConfig, new Pose2d(1, 1, Rotation2d.kZero));

  private static final StructArrayPublisher<Pose3d> componentPoses =
      NetworkTableInstance.getDefault()
          .getStructArrayTopic("zeroed component poses", Pose3d.struct)
          .publish();

  public static void add3DComponentPoses() {
    componentPoses.set(
        new Pose3d[] {
          new Pose3d(
              new Translation3d(
                  0,
                  0,
                  (ElevatorConstants.heightToPercentage
                              .convertBackwards(Subsystems.elevator.getPercentage())
                              .minus(ElevatorConstants.HEIGHT_MIN))
                          .in(Meters)
                      / 3.0),
              new Rotation3d()),
          new Pose3d(
              new Translation3d(
                  0,
                  0,
                  (ElevatorConstants.heightToPercentage
                              .convertBackwards(Subsystems.elevator.getPercentage())
                              .minus(ElevatorConstants.HEIGHT_MIN))
                          .in(Meters)
                      * 2.0
                      / 3.0),
              new Rotation3d()),
          new Pose3d(
              new Translation3d(
                  0,
                  0,
                  (ElevatorConstants.heightToPercentage
                              .convertBackwards(Subsystems.elevator.getPercentage())
                              .minus(ElevatorConstants.HEIGHT_MIN))
                          .in(Meters)
                      * 5
                      / 6),
              new Rotation3d()),
          new Pose3d(
              new Translation3d(
                      0,
                      0,
                      ((ElevatorConstants.heightToPercentage
                                  .convertBackwards(Subsystems.elevator.getPercentage())
                                  .minus(ElevatorConstants.HEIGHT_MIN))
                              .in(Meters))
                          * 5
                          / 6)
                  .plus(new Translation3d(0.275, 0, 0.38125)),
              new Rotation3d(
                  0,
                  Subsystems.wrist.getWristAngle().minus(WristConstants.IDLE_ROTATION).getRadians(),
                  0))
        });
  }

  public static void addRobotHeldGamePieces() {
    addCoral();
  }

  private static void addCoral() {
    var coralPitch =
        Subsystems.wrist
            .getWristAngle()
            .minus(WristConstants.IDLE_ROTATION)
            .plus(Rotation2d.fromDegrees(10))
            .getRadians();
    if (Subsystems.wrist.hasCoral()) {
      Logger.recordOutput(
          "RobotSimulation/Coral",
          new Pose3d[] {
            new Pose3d(
                new Translation3d(
                        Subsystems.drive.getPose().getX(),
                        Subsystems.drive.getPose().getY(),
                        ((ElevatorConstants.heightToPercentage
                                    .convertBackwards(Subsystems.elevator.getPercentage())
                                    .minus(ElevatorConstants.HEIGHT_MIN))
                                .in(Meters))
                            * 5
                            / 6)
                    .plus(
                        new Translation3d(0.275, 0, 0.38125)
                            .rotateBy(new Rotation3d(Subsystems.drive.getPose().getRotation())))
                    .plus(
                        new Translation3d(
                                -Units.inchesToMeters(10)
                                    * Math.cos(coralPitch + Units.degreesToRadians(-20)),
                                0,
                                Units.inchesToMeters(10)
                                    * Math.sin(coralPitch + Units.degreesToRadians(-20)))
                            .rotateBy(new Rotation3d(Subsystems.drive.getPose().getRotation()))),
                new Rotation3d(0, coralPitch, 0)
                    .plus(new Rotation3d(Subsystems.drive.getPose().getRotation())))
          });
    } else {
      Logger.recordOutput("RobotSimulation/Coral", new Pose3d[] {});
    }
  }
}
