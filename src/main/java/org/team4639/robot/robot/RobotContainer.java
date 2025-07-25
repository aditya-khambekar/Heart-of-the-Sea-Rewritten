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
import edu.wpi.first.wpilibj2.command.button.Trigger;
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
import org.team4639.lib.led.subsystem.PhysicalLEDStrip;
import org.team4639.lib.oi.OI;
import org.team4639.lib.oi.RSXboxController;
import org.team4639.lib.util.AllianceFlipUtil;
import org.team4639.lib.util.DriverStationUtil;
import org.team4639.robot.Constants;
import org.team4639.robot.auto.AutoFactory;
import org.team4639.robot.commands.DriveCommands;
import org.team4639.robot.commands.LEDCommands;
import org.team4639.robot.commands.SuperstructureCommands;
import org.team4639.robot.constants.reefscape.FieldConstants;
import org.team4639.robot.constants.robot.IDs;
import org.team4639.robot.statemachine.StateControls;
import org.team4639.robot.subsystems.ControllerRumble;
import org.team4639.robot.subsystems.DashboardOutputs;
import org.team4639.robot.subsystems.LimelightFlash;
import org.team4639.robot.subsystems.drive.Drive;
import org.team4639.robot.subsystems.drive.generated.TunerConstants;
import org.team4639.robot.subsystems.superstructure.Superstructure;
import org.team4639.robot.subsystems.superstructure.elevator.Elevator;
import org.team4639.robot.subsystems.superstructure.elevator.io.ElevatorIOTalonFX;
import org.team4639.robot.subsystems.superstructure.elevator.io.ElevatorIOTalonFXSim;
import org.team4639.robot.subsystems.superstructure.roller.Roller;
import org.team4639.robot.subsystems.superstructure.roller.io.RollerIOSim;
import org.team4639.robot.subsystems.superstructure.roller.io.RollerIOSparkFlex;
import org.team4639.robot.subsystems.superstructure.wrist.Wrist;
import org.team4639.robot.subsystems.superstructure.wrist.io.WristIOSim;
import org.team4639.robot.subsystems.superstructure.wrist.io.WristIOSparkFlex;
import org.team4639.robot.subsystems.vision.*;

/**
 * This class is where the bulk of the robot should be declared. Since Command-based is a
 * "declarative" paradigm, very little robot logic should actually be handled in the {@link Robot}
 * periodic methods (other than the scheduler calls). Instead, the structure of the robot (including
 * subsystems, commands, and button mappings) should be declared here.
 */
public final class RobotContainer {
  // Controller
  public static final RSXboxController driver = OI.driver;
  public static final RSXboxController operator = OI.operator;

  // Dashboard inputs
  private final LoggedDashboardChooser<Command> autoChooser;

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
            new Elevator(
                new ElevatorIOTalonFX(
                    IDs.Superstructure.ELEVATOR_LEFT_FOLLOWER,
                    IDs.Superstructure.ELEVATOR_RIGHT_LEADER));
        Subsystems.wrist =
            new Wrist(
                new WristIOSparkFlex(IDs.Superstructure.WRIST),
                new LaserCanIOHardware(IDs.Superstructure.WRIST_LASERCAN));
        Subsystems.roller = new Roller(new RollerIOSparkFlex(IDs.Superstructure.ROLLER));
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
            new Elevator(
                new ElevatorIOTalonFXSim(
                    IDs.Superstructure.ELEVATOR_LEFT_FOLLOWER,
                    IDs.Superstructure.ELEVATOR_RIGHT_LEADER));
        Subsystems.wrist =
            new Wrist(
                new WristIOSim(IDs.Superstructure.WRIST),
                new LaserCanIOSim(
                    () ->
                        Millimeter.of(
                            SmartDashboard.getBoolean("Has Coral", true) ? 1 : Integer.MAX_VALUE)));
        Subsystems.roller = new Roller(new RollerIOSim(IDs.Superstructure.ROLLER));
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
    Subsystems.superstructure = new Superstructure();
    Subsystems.limelightFlash = new LimelightFlash(VisionConstants.cameraBackName);
    Subsystems.controllerRumble = new ControllerRumble();
    Subsystems.leds = new PhysicalLEDStrip(9, 96);

    // Configure the button bindings
    configureButtonBindings();
    // Anything else that needs to be initialized
    initRobotOperations();
    // initialize state machine
    StateControls.initStateMachine();
    // I truly have no idea why calling this variable instantiates FieldConstants but it works so.
    double x = FieldConstants.fieldLength;

    // Set up auto routines
    autoChooser = new LoggedDashboardChooser<>("Auto Choices", AutoBuilder.buildAutoChooser());

    // Set up SysId routines
    autoChooser.addOption("TEST_1MTR", AutoFactory.TEST_1MTR());
    autoChooser.addOption(
        "(ALGAE_LEFT) MS_G4_ALGH_ALGSC1_ALIJ_ALGSC2", AutoFactory.MS_G4_ALGH_ALGSC1_ALIJ_ALGSC2());
    autoChooser.addOption("(R4-1) RS-F4-E4-D4-C4", AutoFactory.RS_F4_E4_D4_C4());
    autoChooser.addOption("(R3-1) RS-F4-E4-D4", AutoFactory.RS_F4_E4_D4());
    autoChooser.addOption("(R3-2) RS-E4-D4-C4", AutoFactory.RS_E4_D4_C4());
    autoChooser.addOption("(1678R) RS-F4-D4-C4-E4", AutoFactory.RS_F4_D4_C4_E4());
    autoChooser.addOption("(L4-1) LS-I4-J4-K4-L4", AutoFactory.LS_I4_J4_K4_L4());
    autoChooser.addOption("(L3-1) LS-I4-J4-K4", AutoFactory.LS_I4_J4_K4());
    autoChooser.addOption("(L3-2) LS-J4-K4-L4", AutoFactory.LS_J4_K4_L4());
    autoChooser.addOption("(1678L) LS-I4-K4-L4-J4", AutoFactory.LS_I4_K4_L4_J4());
    autoChooser.addOption(
        "Drive Wheel Radius Characterization",
        DriveCommands.Evergreen.wheelRadiusCharacterization());
    autoChooser.addOption(
        "Drive Simple FF Characterization", DriveCommands.Evergreen.feedforwardCharacterization());
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
  }

  private void initRobotOperations() {
    VisionUpdates.addConsumer(Subsystems.drive);
    VisionUpdates.addConsumer(VisionPoses.frontCamerasPoseEstimate);
    SuperstructureCommands.initCommands();
    FieldConstants.init();
  }

  /**
   * Use this method to define your button->command mappings. Buttons can be created by
   * instantiating a {@link GenericHID} or one of its subclasses ({@link
   * edu.wpi.first.wpilibj.Joystick} or {@link XboxController}), and then passing it to a {@link
   * edu.wpi.first.wpilibj2.command.button.JoystickButton}.
   */
  private void configureButtonBindings() {

    Subsystems.superstructure.setDefaultCommand(
        Commands.defer(
            SuperstructureCommands::idle,
            Set.of(
                Subsystems.elevator,
                Subsystems.wrist,
                Subsystems.roller,
                Subsystems.superstructure)));

    // Default command, normal field-relative drive
    Subsystems.drive.setDefaultCommand(
        DriveCommands.Evergreen.joystickDrive(
                () -> -driver.getLeftY(), () -> -driver.getLeftX(), () -> -driver.getRightX())
            .withName("Drive Joystick"));

    Subsystems.leds.setDefaultCommand(LEDCommands.disabled());

    new Trigger(DriverStationUtil::usingDSAlliance).onTrue(LEDCommands.disabled());
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
