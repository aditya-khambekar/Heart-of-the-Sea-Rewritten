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

package org.team4639.robot.robot;

import static edu.wpi.first.units.Units.Millimeter;

import com.pathplanner.lib.auto.AutoBuilder;
import edu.wpi.first.wpilibj.GenericHID;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine;
import java.util.Set;
import org.littletonrobotics.junction.networktables.LoggedDashboardChooser;
import org.team4639.lib.error.Errors;
import org.team4639.lib.io.sensor.lasercan.LaserCanIOHardware;
import org.team4639.lib.io.sensor.lasercan.LaserCanIOSim;
import org.team4639.lib.io.swerve.GyroIO;
import org.team4639.lib.io.swerve.GyroIOPigeon2;
import org.team4639.lib.io.swerve.ModuleIO;
import org.team4639.lib.io.swerve.ModuleIOSim;
import org.team4639.lib.io.swerve.ModuleIOTalonFX;
import org.team4639.lib.io.vision.VisionIO;
import org.team4639.lib.io.vision.VisionIOLimelight;
import org.team4639.lib.io.vision.VisionIOPhotonVisionSim;
import org.team4639.lib.oi.OI;
import org.team4639.lib.statebased.StateMachine;
import org.team4639.lib.util.AllianceFlipUtil;
import org.team4639.robot.Constants;
import org.team4639.robot.auto.AutoFactory;
import org.team4639.robot.commands.DriveCommands;
import org.team4639.robot.commands.SuperstructureCommands;
import org.team4639.robot.constants.FieldConstants;
import org.team4639.robot.constants.IDs;
import org.team4639.robot.modaltriggers.IOTriggers;
import org.team4639.robot.statemachine.States;
import org.team4639.robot.subsystems.DashboardOutputs;
import org.team4639.robot.subsystems.ReefTracker;
import org.team4639.robot.subsystems.drive.Drive;
import org.team4639.robot.subsystems.drive.generated.TunerConstants;
import org.team4639.robot.subsystems.superstructure.Superstructure;
import org.team4639.robot.subsystems.superstructure.elevator.ElevatorSubsystem;
import org.team4639.robot.subsystems.superstructure.elevator.io.ElevatorIOTalonFX;
import org.team4639.robot.subsystems.superstructure.elevator.io.ElevatorIOTalonFXSim;
import org.team4639.robot.subsystems.superstructure.roller.RollerSubsystem;
import org.team4639.robot.subsystems.superstructure.roller.io.RollerIOSim;
import org.team4639.robot.subsystems.superstructure.roller.io.RollerIOSparkFlex;
import org.team4639.robot.subsystems.superstructure.wrist.WristSubsystem;
import org.team4639.robot.subsystems.superstructure.wrist.io.WristIOSim;
import org.team4639.robot.subsystems.superstructure.wrist.io.WristIOSparkFlex;
import org.team4639.robot.subsystems.vision.*;

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
    Errors.addCheck(OI.driver::isConnected, "Driver Controller disconnected");
    Errors.addCheck(OI.operator::isConnected, "Operator Controller disconnected");
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

        Subsystems.elevator =
            new ElevatorSubsystem(new ElevatorIOTalonFX(IDs.ELEVATOR_LEFT, IDs.ELEVATOR_RIGHT));
        Subsystems.wrist =
            new WristSubsystem(
                new WristIOSparkFlex(IDs.WRIST), new LaserCanIOHardware(IDs.WRIST_LASERCAN));
        Subsystems.roller = new RollerSubsystem(new RollerIOSparkFlex(IDs.ROLLER));
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
        SmartDashboard.putBoolean("Has Coral", false);
        Subsystems.elevator =
            new ElevatorSubsystem(new ElevatorIOTalonFXSim(IDs.ELEVATOR_LEFT, IDs.ELEVATOR_RIGHT));
        Subsystems.wrist =
            new WristSubsystem(
                new WristIOSim(IDs.WRIST),
                new LaserCanIOSim(
                    () ->
                        Millimeter.of(
                            SmartDashboard.getBoolean("Has Coral", true) ? 1 : Integer.MAX_VALUE)));
        Subsystems.roller = new RollerSubsystem(new RollerIOSim(IDs.ROLLER));
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
        break;
    }

    Subsystems.dashboardOutputs = new DashboardOutputs();
    Subsystems.reefTracker = new ReefTracker();
    Subsystems.superstructure = new Superstructure();

    VisionUpdates.addConsumer(Subsystems.drive);
    VisionUpdates.addConsumer(VisionPoses.frontCamerasPoseEstimate);

    // Configure the button bindings
    configureButtonBindings();
    SuperstructureCommands.initCommands();
    FieldConstants.init();
    FieldConstants.initAlgaeLocations();
    Subsystems.reefTracker.resetReefTracker();
    States.initStaticStates();
    // I truly have no idea why calling this variable instantiates FieldConstants but it works so.
    double x = FieldConstants.fieldLength;
    var y = IOTriggers.hasDriverJoystickInput;

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

    StateMachine.setState(States.IDLE);
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
                () -> -driver.getRightX())
            .withName("Drive Joystick"));
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
