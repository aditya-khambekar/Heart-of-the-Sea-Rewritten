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

package org.team4639._robot;

import com.pathplanner.lib.auto.AutoBuilder;
import edu.wpi.first.wpilibj.GenericHID;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine;
import org.littletonrobotics.junction.networktables.LoggedDashboardChooser;
import org.team4639.Constants;
import org.team4639._lib.oi.OI;
import org.team4639.commands.DriveCommands;
import org.team4639.constants.FieldConstants;
import org.team4639.modaltriggers.DriveTriggers;
import org.team4639.subsystems.drive.Drive;
import org.team4639.subsystems.drive.GyroIO;
import org.team4639.subsystems.drive.GyroIOPigeon2;
import org.team4639.subsystems.drive.ModuleIO;
import org.team4639.subsystems.drive.ModuleIOSim;
import org.team4639.subsystems.drive.ModuleIOTalonFX;
import org.team4639.subsystems.drive.generated.TunerConstants;
import org.team4639.subsystems.elevator.Elevator;
import org.team4639.subsystems.elevator.ElevatorIO;
import org.team4639.subsystems.elevator.ElevatorIOHardware;
import org.team4639.subsystems.elevator.ElevatorIOSim;
import org.team4639.subsystems.scoring.Scoring;
import org.team4639.subsystems.scoring.ScoringIO;
import org.team4639.subsystems.scoring.ScoringIOHardware;
import org.team4639.subsystems.scoring.ScoringIOSim;
import org.team4639.subsystems.vision.*;
import org.team4639.subsystems.vision.VisionIO;
import org.team4639.subsystems.vision.VisionIOLimelight;
import org.team4639.subsystems.vision.VisionIOPhotonVisionSim;

/**
 * This class is where the bulk of the robot should be declared. Since Command-based is a
 * "declarative" paradigm, very little robot logic should actually be handled in the {@link Robot}
 * periodic methods (other than the scheduler calls). Instead, the structure of the robot (including
 * subsystems, commands, and button mappings) should be declared here.
 */
public class RobotContainer {
  // Controller
  private final CommandXboxController driver = OI.driver;
  private final CommandXboxController operator = OI.operator;

  // Dashboard inputs
  private final LoggedDashboardChooser<Command> autoChooser;

  /** The container for the robot. Contains subsystems, OI devices, and commands. */
  public RobotContainer() {
    FieldConstants.Reef.init();
    switch (Constants.currentMode) {
      case REAL:
        // Real robot, instantiate hardware IO implementations
        Subsystems.drive =
            new Drive(
                new GyroIOPigeon2(),
                new ModuleIOTalonFX(TunerConstants.FrontLeft),
                new ModuleIOTalonFX(TunerConstants.FrontRight),
                new ModuleIOTalonFX(TunerConstants.BackLeft),
                new ModuleIOTalonFX(TunerConstants.BackRight));

        Subsystems.vision =
            new Vision(
                VisionUpdates.getInstance(),
                new VisionIOLimelight(
                    VisionConstants.cameraLeftName, Subsystems.drive::getRotation),
                new VisionIOLimelight(
                    VisionConstants.cameraRightName, Subsystems.drive::getRotation));

        Subsystems.elevator = new Elevator(new ElevatorIOHardware());
        Subsystems.scoring = new Scoring(new ScoringIOHardware());
        break;

      case SIM:
        // Sim robot, instantiate physics sim IO implementations
        Subsystems.drive =
            new Drive(
                new GyroIO() {},
                new ModuleIOSim(TunerConstants.FrontLeft),
                new ModuleIOSim(TunerConstants.FrontRight),
                new ModuleIOSim(TunerConstants.BackLeft),
                new ModuleIOSim(TunerConstants.BackRight));

        Subsystems.vision =
            new Vision(
                VisionUpdates.getInstance(),
                new VisionIOPhotonVisionSim(
                    VisionConstants.cameraLeftName,
                    VisionConstants.robotToCameraLeft,
                    Subsystems.drive::getPose),
                new VisionIOPhotonVisionSim(
                    VisionConstants.cameraRightName,
                    VisionConstants.robotToCameraRight,
                    Subsystems.drive::getPose));

        Subsystems.elevator = new Elevator(new ElevatorIOSim());
        Subsystems.scoring = new Scoring(new ScoringIOSim());
        break;

      default:
        // Replayed robot, disable IO implementations
        Subsystems.drive =
            new Drive(
                new GyroIO() {},
                new ModuleIO() {},
                new ModuleIO() {},
                new ModuleIO() {},
                new ModuleIO() {});

        Subsystems.vision =
            new Vision(VisionUpdates.getInstance(), new VisionIO() {}, new VisionIO() {});

        Subsystems.elevator = new Elevator(new ElevatorIO() {});
        Subsystems.scoring = new Scoring(new ScoringIO() {});
        break;
    }

    VisionUpdates.addConsumer(Subsystems.drive);
    VisionUpdates.addConsumer(VisionPoses.frontCamerasPoseEstimate);

    // Set up auto routines
    autoChooser = new LoggedDashboardChooser<>("Auto Choices", AutoBuilder.buildAutoChooser());

    // Set up SysId routines
    autoChooser.addOption(
        "Drive Wheel Radius Characterization",
        DriveCommands.wheelRadiusCharacterization(Subsystems.drive));
    autoChooser.addOption(
        "Drive Simple FF Characterization",
        DriveCommands.feedforwardCharacterization(Subsystems.drive));
    autoChooser.addOption(
        "Drive SysId (Quasistatic Forward)",
        Subsystems.drive.sysIdQuasistatic(SysIdRoutine.Direction.kForward));
    autoChooser.addOption(
        "Drive SysId (Quasistatic Reverse)",
        Subsystems.drive.sysIdQuasistatic(SysIdRoutine.Direction.kReverse));
    autoChooser.addOption(
        "Drive SysId (Dynamic Forward)",
        Subsystems.drive.sysIdDynamic(SysIdRoutine.Direction.kForward));
    autoChooser.addOption(
        "Drive SysId (Dynamic Reverse)",
        Subsystems.drive.sysIdDynamic(SysIdRoutine.Direction.kReverse));

    // Configure the button bindings
    configureButtonBindings();
  }

  /**
   * Use this method to define your button->command mappings. Buttons can be created by
   * instantiating a {@link GenericHID} or one of its subclasses ({@link
   * edu.wpi.first.wpilibj.Joystick} or {@link XboxController}), and then passing it to a {@link
   * edu.wpi.first.wpilibj2.command.button.JoystickButton}.
   */
  private void configureButtonBindings() {
    // Default command, normal field-relative drive
    Subsystems.drive.setDefaultCommand(
        DriveCommands.joystickDrive(
            Subsystems.drive,
            () -> -driver.getLeftY(),
            () -> -driver.getLeftX(),
            () -> -driver.getRightX()));

    Subsystems.elevator.setDefaultCommand(Subsystems.elevator.runToSetpoint(0.5));

    DriveTriggers.closeToLeftStation.whileTrue(
        DriveCommands.joystickDriveAtAngle(
            Subsystems.drive,
            () -> -driver.getLeftY(),
            () -> -driver.getLeftX(),
            FieldConstants.CoralStation.leftCenterFace::getRotation));

    DriveTriggers.closeToRightStation.whileTrue(
        DriveCommands.joystickDriveAtAngle(
            Subsystems.drive,
            () -> -driver.getLeftY(),
            () -> -driver.getLeftX(),
            FieldConstants.CoralStation.rightCenterFace::getRotation));
  }

  /**
   * Use this to pass the autonomous command to the main {@link Robot} class.
   *
   * @return the command to run in autonomous
   */
  public Command getAutonomousCommand() {
    return autoChooser.get();
  }
}
