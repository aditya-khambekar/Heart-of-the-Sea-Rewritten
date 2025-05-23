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

import au.grapplerobotics.LaserCan;
import au.grapplerobotics.simulation.MockLaserCan;
import com.ctre.phoenix6.hardware.TalonFX;
import com.pathplanner.lib.auto.AutoBuilder;
import com.revrobotics.spark.SparkLowLevel;
import com.revrobotics.spark.SparkMax;
import edu.wpi.first.wpilibj.GenericHID;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.RobotModeTriggers;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine;
import java.util.Set;
import org.littletonrobotics.junction.networktables.LoggedDashboardChooser;
import org.team4639.Constants;
import org.team4639._lib.io.swerve.GyroIO;
import org.team4639._lib.io.swerve.GyroIOPigeon2;
import org.team4639._lib.io.swerve.ModuleIO;
import org.team4639._lib.io.swerve.ModuleIOSim;
import org.team4639._lib.io.swerve.ModuleIOTalonFX;
import org.team4639._lib.io.vision.VisionIO;
import org.team4639._lib.io.vision.VisionIOLimelight;
import org.team4639._lib.io.vision.VisionIOPhotonVisionSim;
import org.team4639._lib.oi.OI;
import org.team4639._lib.util.AllianceFlipUtil;
import org.team4639.auto.AutoFactory;
import org.team4639.commands.DriveCommands;
import org.team4639.constants.FieldConstants;
import org.team4639.constants.IDs;
import org.team4639.modaltriggers.DriveTriggers;
import org.team4639.modaltriggers.IOTriggers;
import org.team4639.modaltriggers.VisionTriggers;
import org.team4639.subsystems.DashboardOutputs;
import org.team4639.subsystems.drive.Drive;
import org.team4639.subsystems.drive.generated.TunerConstants;
import org.team4639.subsystems.elevator.Elevator;
import org.team4639.subsystems.elevator.ElevatorConstants;
import org.team4639.subsystems.elevator.ElevatorIO;
import org.team4639.subsystems.elevator.ElevatorIOHardware;
import org.team4639.subsystems.elevator.ElevatorIOSim;
import org.team4639.subsystems.scoring.Scoring;
import org.team4639.subsystems.scoring.ScoringIO;
import org.team4639.subsystems.scoring.ScoringIOHardware;
import org.team4639.subsystems.scoring.ScoringIOSim;
import org.team4639.subsystems.vision.*;

/**
 * This class is where the bulk of the robot should be declared. Since Command-based is a
 * "declarative" paradigm, very little robot logic should actually be handled in the {@link Robot}
 * periodic methods (other than the scheduler calls). Instead, the structure of the robot (including
 * subsystems, commands, and button mappings) should be declared here.
 */
public class RobotContainer {
  // Controller
  public static final CommandXboxController driver = OI.driver;
  public static final CommandXboxController operator = OI.operator;

  // Dashboard inputs
  private final LoggedDashboardChooser<Command> autoChooser;

  private final boolean dummySuperstructure = true;

  protected final Field2d field = new Field2d();

  /** The container for the robot. Contains subsystems, OI devices, and commands. */
  public RobotContainer() {
    FieldConstants.init();
    // I truly have no idea why calling this variable instantiates FieldConstants but it works so.
    double x = FieldConstants.fieldLength;
    var y = IOTriggers.hasDriverJoystickInput;
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
                    VisionConstants.cameraRightName, Subsystems.drive::getRotation),
                new VisionIOLimelight(
                    VisionConstants.cameraLeftName, Subsystems.drive::getRotation));

        if (dummySuperstructure) {
          Subsystems.elevator = new Elevator(new ElevatorIO() {});
          Subsystems.scoring = new Scoring(new ScoringIO() {});
        } else {
          Subsystems.elevator =
              new Elevator(
                  new ElevatorIOHardware(
                      new TalonFX(IDs.ELEVATOR_LEFT), new TalonFX(IDs.ELEVATOR_RIGHT)));
          Subsystems.scoring =
              new Scoring(
                  new ScoringIOHardware(
                      new SparkMax(IDs.SCORING_MOTOR, SparkLowLevel.MotorType.kBrushless),
                      new LaserCan(IDs.INTAKE_LASERCAN_ID)));
        }
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

        // We flip the vision pose in the VisionIO if Red Alliance, so if we are on red we need to
        // flip the poses back when we feed them in to maintain the VisionIO being given the Blue
        // Alliance pose.
        Subsystems.vision =
            new Vision(
                VisionUpdates.getInstance(),
                new VisionIOPhotonVisionSim(
                    VisionConstants.cameraLeftName,
                    VisionConstants.robotToCameraLeft,
                    () -> AllianceFlipUtil.flipIfRedAlliance(Subsystems.drive.getPose())),
                new VisionIOPhotonVisionSim(
                    VisionConstants.cameraRightName,
                    VisionConstants.robotToCameraRight,
                    () -> AllianceFlipUtil.flipIfRedAlliance(Subsystems.drive.getPose())),
                new VisionIOPhotonVisionSim(
                    VisionConstants.cameraBackName,
                    VisionConstants.robotToCameraBack,
                    () -> AllianceFlipUtil.flipIfRedAlliance(Subsystems.drive.getPose())));

        Subsystems.elevator =
            new Elevator(
                new ElevatorIOSim(new TalonFX(IDs.ELEVATOR_LEFT), new TalonFX(IDs.ELEVATOR_RIGHT)));
        Subsystems.scoring =
            new Scoring(
                new ScoringIOSim(
                    new SparkMax(IDs.SCORING_MOTOR, SparkLowLevel.MotorType.kBrushless),
                    new MockLaserCan()));
        Subsystems.dashboardOutputs = new DashboardOutputs();
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
        Subsystems.dashboardOutputs = new DashboardOutputs();
        break;
    }

    VisionUpdates.addConsumer(Subsystems.drive);
    VisionUpdates.addConsumer(VisionPoses.frontCamerasPoseEstimate);

    // Set up auto routines
    autoChooser = new LoggedDashboardChooser<>("Auto Choices", AutoBuilder.buildAutoChooser());

    // Set up SysId routines
    autoChooser.addOption("TEST_1MTR", AutoFactory.TEST_1MTR());
    autoChooser.addOption(
        "MS_G_ALGH_ALGSC1_ALIJ_ALGSC2", AutoFactory.MS_G_ALGH_ALGSC1_ALIJ_ALGSC2());
    autoChooser.addOption("RS-F-E-D-C", AutoFactory.RS_F_E_D_C());
    autoChooser.addOption("RS-F-E-D", AutoFactory.RS_F_E_D());
    autoChooser.addOption("RS-E-D-C", AutoFactory.RS_E_D_C());
    autoChooser.addOption("LS-I-J-K-L", AutoFactory.LS_I_J_K_L());
    autoChooser.addOption("LS-I-J-K", AutoFactory.LS_I_J_K());
    autoChooser.addOption("LS-J-K-L", AutoFactory.LS_J_K_L());
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
    driver
        .leftStick()
        .onTrue(
            Commands.defer(
                () ->
                    Commands.runOnce(
                        () -> {
                          if (RobotMode.isComp()) RobotMode.setRobotMode((byte) 0b1);
                          else RobotMode.setRobotMode((byte) 0b0);
                        }),
                Set.of()));

    // Default command, normal field-relative drive
    Subsystems.drive.setDefaultCommand(
        DriveCommands.joystickDrive(
            Subsystems.drive,
            () -> -driver.getLeftY(),
            () -> -driver.getLeftX(),
            () -> -driver.getRightX()));

    Subsystems.elevator.setDefaultCommand(
        Subsystems.elevator.defer(
            () ->
                Subsystems.elevator
                    .runToSetpoint(
                        ElevatorConstants.ProportionToPosition.convert(
                            ElevatorConstants.Setpoints.IDLE_PROPORTION))
                    .withName("Default")));

    DriveTriggers.closeToLeftStation
        .and(RobotMode::isComp)
        .and(RobotModeTriggers.teleop())
        .and(VisionTriggers.visionIsActive())
        .and(IOTriggers.hasDriverRotationalInput.negate())
        .whileTrue(
            DriveCommands.vectorCoralStationAlignLeft(
                Subsystems.drive, () -> -driver.getLeftY(), () -> -driver.getLeftX()));

    DriveTriggers.closeToRightStation
        .and(RobotMode::isComp)
        .and(RobotModeTriggers.teleop())
        .and(VisionTriggers.visionIsActive())
        .and(IOTriggers.hasDriverRotationalInput.negate())
        .whileTrue(
            DriveCommands.vectorCoralStationAlignRight(
                Subsystems.drive, () -> -driver.getLeftY(), () -> -driver.getLeftX()));

    //    SuperstructureTriggers.intake
    //        .and(RobotMode::isComp)
    //        .and(RobotModeTriggers.teleop())
    //        .whileTrue(Subsystems.elevator.defer(SuperstructureCommands::intakeCoral));
    //
    //    SuperstructureTriggers.raiseElevator
    //        .and(RobotMode::isComp)
    //        .and(RobotModeTriggers.teleop())
    //        .whileTrue(
    //            Subsystems.elevator.defer(
    //                () ->
    //                    Subsystems.elevator
    //                        .runToSetpoint(
    //                            ElevatorConstants.ProportionToPosition.convert(
    //                                ElevatorConstants.Setpoints.SCORE_READY_POSITION))
    //                        .withName("Raise Elevator")));

    driver
        .rightTrigger()
        .and(RobotModeTriggers.teleop())
        .and(RobotMode::isComp)
        .and(VisionTriggers.visionIsActive())
        .whileTrue(Subsystems.drive.defer(() -> DriveCommands.reefAlign(Subsystems.drive)));

    driver
        .leftBumper()
        .and(RobotModeTriggers.teleop())
        .and(RobotMode::isComp)
        .and(VisionTriggers.visionIsActive())
        .whileTrue(Subsystems.drive.defer(() -> DriveCommands.reefAlignLeft(Subsystems.drive)));

    driver
        .rightBumper()
        .and(RobotModeTriggers.teleop())
        .and(RobotMode::isComp)
        .and(VisionTriggers.visionIsActive())
        .whileTrue(Subsystems.drive.defer(() -> DriveCommands.reefAlignRight(Subsystems.drive)));

    driver.povUp().and(RobotMode::isManual).whileTrue(Subsystems.elevator.runVelocity(5.0));
    driver.povDown().and(RobotMode::isManual).whileTrue(Subsystems.elevator.runVelocity(-5.0));

    driver.a().and(RobotMode::isManual).whileTrue(Subsystems.scoring.runMotor(0.5));
    driver.b().and(RobotMode::isManual).whileTrue(Subsystems.scoring.runMotor(-0.5));
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
