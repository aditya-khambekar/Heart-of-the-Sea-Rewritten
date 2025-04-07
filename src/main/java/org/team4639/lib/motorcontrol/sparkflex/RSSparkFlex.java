package org.team4639.lib.motorcontrol.sparkflex;

import com.revrobotics.REVLibError;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.sim.SparkFlexSim;
import com.revrobotics.spark.SparkAbsoluteEncoder;
import com.revrobotics.spark.SparkAnalogSensor;
import com.revrobotics.spark.SparkBase;
import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkBase.ResetMode;
import com.revrobotics.spark.SparkClosedLoopController;
import com.revrobotics.spark.SparkFlex;
import com.revrobotics.spark.SparkLimitSwitch;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.SparkSim;
import com.revrobotics.spark.config.AbsoluteEncoderConfig;
import com.revrobotics.spark.config.AnalogSensorConfig;
import com.revrobotics.spark.config.ClosedLoopConfig;
import com.revrobotics.spark.config.EncoderConfig;
import com.revrobotics.spark.config.ExternalEncoderConfig;
import com.revrobotics.spark.config.LimitSwitchConfig;
import com.revrobotics.spark.config.SoftLimitConfig;
import com.revrobotics.spark.config.SparkBaseConfig;
import com.revrobotics.spark.config.SparkFlexConfig;
import edu.wpi.first.math.numbers.N1;
import edu.wpi.first.math.numbers.N2;
import edu.wpi.first.math.system.LinearSystem;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.math.system.plant.LinearSystemId;
import edu.wpi.first.units.measure.Voltage;
import edu.wpi.first.util.sendable.Sendable;
import edu.wpi.first.util.sendable.SendableBuilder;
import edu.wpi.first.wpilibj.RobotBase;
import edu.wpi.first.wpilibj.simulation.*;
import edu.wpi.first.wpilibj2.command.Subsystem;
import org.littletonrobotics.junction.AutoLogOutput;
import org.team4639.lib.motorcontrol.generic.NeutralMode;
import org.team4639.lib.motorcontrol.generic.RSGenericMotorController;
import org.team4639.lib.subsystem.RSSubsystem;
import org.team4639.lib.subsystem.Updatable;

/**
 * Wrapper class for a {@link SparkFlex} with automatic SmartDashboard and AdvantageScope logging.
 * Make sure to call {@link RSSparkFlex#update()} in the {@link Subsystem#periodic()} method, or
 * call the {@link RSSubsystem#addMember(Updatable)} method in an RSSubsystem.
 */
public class RSSparkFlex implements RSGenericMotorController, Sendable {
  protected int ID;

  protected SparkFlex sparkFlex;

  protected SparkFlexConfig config;
  protected ResetMode resetMode;
  protected PersistMode persistMode;
  protected String name;

  protected boolean isSimulation;
  protected DCMotorSim motorSim;
  protected SparkFlexSim sparkFlexSim;
  protected LinearSystem<N2, N1, N2> plant;
  protected DCMotor gearbox;

  public RSSparkFlex(int ID, MotorType motorType) {
    this.ID = ID;
    this.sparkFlex = new SparkFlex(ID, motorType);
    this.config = new SparkFlexConfig();
    this.resetMode = ResetMode.kResetSafeParameters;
    this.persistMode = PersistMode.kPersistParameters;
    this.name = "RSSparkFlex" + this;

    isSimulation = RobotBase.isSimulation();
    gearbox = DCMotor.getNeoVortex(1);
    // default of a NEO Vortex attached to nothing
    motorSim =
        new DCMotorSim(
            (plant = LinearSystemId.createDCMotorSystem(gearbox, 0.4445 * Math.pow(0.025, 2), 1.0)),
            gearbox,
            0.0);
    sparkFlexSim = new SparkFlexSim(sparkFlex, gearbox);
  }

  public RSSparkFlex withName(String name) {
    this.name = name;
    return this;
  }

  public void setName(String name) {
    this.name = name;
  }

  public int getID() {
    return ID;
  }

  public SparkFlex getRawSparkFlex() {
    return sparkFlex;
  }

  public SparkFlexConfig getRawConfig() {
    return config;
  }

  private void reconfigure() {
    sparkFlex.configure(config, resetMode, persistMode);
  }

  /**
   * Returns an object for interfacing with an external quadrature encoder
   *
   * @return An object for interfacing with an external quadrature encoder
   */
  public RelativeEncoder getExternalEncoder() {
    return sparkFlex.getExternalEncoder();
  }

  /**
   * Common interface for setting the speed of a motor controller.
   *
   * @param speed The speed to set. Value should be between -1.0 and 1.0.
   */
  public void set(double speed) {
    sparkFlex.set(speed);
  }

  /**
   * Sets the voltage output of the SpeedController. This is equivillant to a call to
   * SetReference(output, rev::ControlType::kVoltage). The behavior of this call differs slightly
   * from the WPILib documetation for this call since the device internally sets the desired voltage
   * (not a compensation value). That means that this *can* be a 'set-and-forget' call.
   *
   * @param outputVolts The voltage to output.
   */
  public void setVoltage(double outputVolts) {
    sparkFlex.setVoltage(outputVolts);
  }

  /**
   * Sets the voltage output of the MotorController. Compensates for the current bus voltage to
   * ensure that the desired voltage is output even if the battery voltage is below 12V - highly
   * useful when the voltage outputs are "meaningful" (e.g. they come from a feedforward
   * calculation).
   *
   * <p>NOTE: This function *must* be called regularly in order for voltage compensation to work
   * properly - unlike the ordinary set function, it is not "set it and forget it."
   *
   * @param outputVoltage The voltage to output.
   */
  public void setVoltage(Voltage outputVoltage) {
    sparkFlex.setVoltage(outputVoltage);
  }

  /**
   * Common interface for getting the current set speed of a motor controller.
   *
   * @return The current set speed. Value is between -1.0 and 1.0.
   */
  public double get() {
    if (isSimulation) return sparkFlexSim.getAppliedOutput();
    return sparkFlex.get();
  }

  /**
   * Common interface for inverting direction of a motor controller.
   *
   * @param isInverted The state of inversion true is inverted.
   */
  public void setInverted(boolean isInverted) {
    config.inverted(isInverted);
    reconfigure();
  }

  /**
   * Common interface for returning if a motor controller is in the inverted state or not.
   *
   * @return isInverted The state of the inversion true is inverted.
   */
  public boolean getInverted() {
    return sparkFlex.configAccessor.getInverted();
  }

  /** Disable the motor controller. */
  public void disable() {
    sparkFlex.disable();
  }

  /**
   * Stops motor movement. Motor can be moved again by calling set without having to re-enable the
   * motor.
   */
  public void stopMotor() {
    sparkFlex.stopMotor();
  }

  @Override
  public void setNeutralMode(NeutralMode mode) {
    switch (mode) {
      case BRAKE:
        this.setIdleMode(SparkBaseConfig.IdleMode.kBrake);
        break;
      case COAST:
        this.setIdleMode(SparkBaseConfig.IdleMode.kCoast);
        break;
    }
  }

  /**
   * Set the configuration for the SPARK.
   *
   * <p>If {@code resetMode} is {@link ResetMode#kResetSafeParameters}, this method will reset safe
   * writable parameters to their default values before setting the given configuration. The
   * following parameters will not be reset by this action: CAN ID, Motor Type, Idle Mode, PWM Input
   * Deadband, and Duty Cycle Offset.
   *
   * <p>If {@code persistMode} is {@link PersistMode#kPersistParameters}, this method will save all
   * parameters to the SPARK's non-volatile memory after setting the given configuration. This will
   * allow parameters to persist across power cycles.
   *
   * @param config The desired SPARK configuration
   * @param resetMode Whether to reset safe parameters before setting the configuration
   * @param persistMode Whether to persist the parameters after setting the configuration
   * @return {@link REVLibError#kOk} if successful
   */
  public REVLibError configure(
      SparkBaseConfig config, ResetMode resetMode, PersistMode persistMode) {
    return sparkFlex.configure(config, resetMode, persistMode);
  }

  /**
   * Set the configuration for the SPARK without waiting for a response.
   *
   * <p>If {@code resetMode} is {@link ResetMode#kResetSafeParameters}, this method will reset safe
   * writable parameters to their default values before setting the given configuration. The
   * following parameters will not be reset by this action: CAN ID, Motor Type, Idle Mode, PWM Input
   * Deadband, and Duty Cycle Offset.
   *
   * <p>If {@code persistMode} is {@link PersistMode#kPersistParameters}, this method will save all
   * parameters to the SPARK's non-volatile memory after setting the given configuration. This will
   * allow parameters to persist across power cycles.
   *
   * <p>NOTE: This method will immediately return {@link REVLibError#kOk} and the action will be
   * done in the background. Any errors that occur will be reported to the driver station.
   *
   * @param config The desired SPARK configuration
   * @param resetMode Whether to reset safe parameters before setting the configuration
   * @param persistMode Whether to persist the parameters after setting the configuration
   * @return {@link REVLibError#kOk}
   */
  public REVLibError configureAsync(
      SparkBaseConfig config, ResetMode resetMode, PersistMode persistMode) {
    return sparkFlex.configureAsync(config, resetMode, persistMode);
  }

  /**
   * Returns an object for interfacing with the encoder connected to the encoder pins or front port
   * of the SPARK MAX or the motor interface of the SPARK Flex.
   *
   * @return An object for interfacing with an encoder
   */
  public RelativeEncoder getEncoder() {
    return sparkFlex.getEncoder();
  }

  /**
   * Returns an object for interfacing with a connected analog sensor.
   *
   * @return An object for interfacing with a connected analog sensor.
   */
  public SparkAnalogSensor getAnalog() {
    return sparkFlex.getAnalog();
  }

  /**
   * Returns an object for interfacing with a connected absolute encoder.
   *
   * @return An object for interfacing with a connected absolute encoder
   */
  public SparkAbsoluteEncoder getAbsoluteEncoder() {
    return sparkFlex.getAbsoluteEncoder();
  }

  /**
   * @return An object for interfacing with the integrated closed loop controller.
   */
  public SparkClosedLoopController getClosedLoopController() {
    return sparkFlex.getClosedLoopController();
  }

  public void setCurrentLimit(int stallLimitAmps) {
    config.smartCurrentLimit(stallLimitAmps);
  }

  /**
   * Returns an object for interfacing with the forward limit switch connected to the appropriate
   * pins on the data port.
   *
   * @return An object for interfacing with the forward limit switch.
   */
  public SparkLimitSwitch getForwardLimitSwitch() {
    return sparkFlex.getForwardLimitSwitch();
  }

  /**
   * Returns an object for interfacing with the reverse limit switch connected to the appropriate
   * pins on the data port.
   *
   * @return An object for interfacing with the reverse limit switch.
   */
  public SparkLimitSwitch getReverseLimitSwitch() {
    return sparkFlex.getReverseLimitSwitch();
  }

  /**
   * Resume follower mode if the SPARK has a valid follower mode config.
   *
   * <p>NOTE: Follower mode will start automatically upon configuring follower mode. If the SPARK
   * experiences a power cycle and has follower mode configured, follower mode will automatically
   * restart. This method is only useful after {@link #pauseFollowerMode()} has been called.
   *
   * @return {@link REVLibError#kOk} if successful
   */
  public REVLibError resumeFollowerMode() {
    return sparkFlex.resumeFollowerMode();
  }

  /**
   * Resume follower mode if the SPARK has a valid follower mode config without waiting for a
   * response.
   *
   * <p>NOTE: Follower mode will start automatically upon configuring follower mode. If the SPARK
   * experiences a power cycle and has follower mode configured, follower mode will automatically
   * restart. This method is only useful after {@link #pauseFollowerMode()} has been called.
   *
   * <p>NOTE: This method will immediately return {@link REVLibError#kOk} and the action will be
   * done in the background. Any errors that occur will be reported to the driver station.
   *
   * @return {@link REVLibError#kOk}
   * @see #resumeFollowerMode()
   */
  public REVLibError resumeFollowerModeAsync() {
    return sparkFlex.resumeFollowerModeAsync();
  }

  /**
   * Pause follower mode.
   *
   * <p>NOTE: If the SPARK experiences a power cycle and has follower mode configured, follower mode
   * will automatically restart.
   *
   * @return {@link REVLibError#kOk} if successful
   */
  public REVLibError pauseFollowerMode() {
    return sparkFlex.pauseFollowerMode();
  }

  /**
   * Pause follower mode without waiting for a response
   *
   * <p>NOTE: If the SPARK experiences a power cycle and has follower mode configured, follower mode
   * will automatically restart.
   *
   * <p>NOTE: This method will immediately return {@link REVLibError#kOk} and the action will be
   * done in the background. Any errors that occur will be reported to the driver station.
   *
   * @return {@link REVLibError#kOk}
   * @see #pauseFollowerMode()
   */
  public REVLibError pauseFollowerModeAsync() {
    return sparkFlex.pauseFollowerModeAsync();
  }

  /**
   * Returns whether the controller is following another controller
   *
   * @return True if this device is following another controller false otherwise
   */
  public boolean isFollower() {
    return sparkFlex.isFollower();
  }

  /**
   * Get whether the SPARK has one or more active faults.
   *
   * @return true if there is an active fault
   * @see #getFaults()
   */
  public boolean hasActiveFault() {
    return sparkFlex.hasActiveFault();
  }

  /**
   * Get whether the SPARK has one or more sticky faults.
   *
   * @return true if there is a sticky fault
   * @see #getStickyFaults()
   */
  public boolean hasStickyFault() {
    return sparkFlex.hasStickyFault();
  }

  /**
   * Get whether the SPARK has one or more active warnings.
   *
   * @return true if there is an active warning
   * @see #getWarnings()
   */
  public boolean hasActiveWarning() {
    return sparkFlex.hasActiveWarning();
  }

  /**
   * Get whether the SPARK has one or more sticky warnings.
   *
   * @return true if there is a sticky warning
   * @see #getStickyWarnings()
   */
  public boolean hasStickyWarning() {
    return sparkFlex.hasStickyWarning();
  }

  /**
   * Get the active faults that are currently present on the SPARK. Faults are fatal errors that
   * prevent the motor from running.
   *
   * @return A struct with each fault and their active value
   */
  public SparkBase.Faults getFaults() {
    return sparkFlex.getFaults();
  }

  /**
   * Get the sticky faults that were present on the SPARK at one point since the sticky faults were
   * last cleared. Faults are fatal errors that prevent the motor from running.
   *
   * <p>Sticky faults can be cleared with {@link SparkBase#clearFaults()}.
   *
   * @return A struct with each fault and their sticky value
   */
  public SparkBase.Faults getStickyFaults() {
    return sparkFlex.getStickyFaults();
  }

  /**
   * Get the active warnings that are currently present on the SPARK. Warnings are non-fatal errors.
   *
   * @return A struct with each warning and their active value
   */
  public SparkBase.Warnings getWarnings() {
    return sparkFlex.getWarnings();
  }

  /**
   * Get the sticky warnings that were present on the SPARK at one point since the sticky warnings
   * were last cleared. Warnings are non-fatal errors.
   *
   * <p>Sticky warnings can be cleared with {@link SparkBase#clearFaults()}.
   *
   * @return A struct with each warning and their sticky value
   */
  public SparkBase.Warnings getStickyWarnings() {
    return sparkFlex.getStickyWarnings();
  }

  /**
   * @return The voltage fed into the motor controller.
   */
  @AutoLogOutput(key = "{name}BusVoltage")
  public double getBusVoltage() {
    if (isSimulation) return sparkFlexSim.getBusVoltage();
    return sparkFlex.getBusVoltage();
  }

  /**
   * Simulation note: this value will not be updated during simulation unless {@link
   * SparkSim#iterate} is called
   *
   * @return The motor controller's applied output duty cycle.
   */
  @AutoLogOutput(key = "{name}AppliedOutput")
  public double getAppliedOutput() {
    if (isSimulation) return sparkFlexSim.getAppliedOutput();
    return sparkFlex.getAppliedOutput();
  }

  /**
   * @return The motor controller's output current in Amps.
   */
  @AutoLogOutput(key = "{name}OutputCurrent")
  public double getOutputCurrent() {
    if (isSimulation) return sparkFlexSim.getMotorCurrent();
    return sparkFlex.getOutputCurrent();
  }

  /**
   * @return The motor temperature in Celsius.
   */
  @AutoLogOutput(key = "{name}MotorTemperature")
  public double getMotorTemperature() {
    if (isSimulation) return -1.0;
    return sparkFlex.getMotorTemperature();
  }

  /**
   * Clears all sticky faults.
   *
   * @return {@link REVLibError#kOk} if successful
   */
  public REVLibError clearFaults() {
    return sparkFlex.clearFaults();
  }

  /**
   * Sets the timeout duration for waiting for CAN responses from the device.
   *
   * @param milliseconds The timeout in milliseconds.
   * @return {@link REVLibError#kOk} if successful
   */
  public REVLibError setCANTimeout(int milliseconds) {
    return sparkFlex.setCANTimeout(milliseconds);
  }

  /**
   * All device errors are tracked on a per thread basis for all devices in that thread. This is
   * meant to be called immediately following another call that has the possibility of returning an
   * error to validate if an error has occurred.
   *
   * @return the last error that was generated.
   */
  public REVLibError getLastError() {
    return sparkFlex.getLastError();
  }

  /** Closes the SPARK Controller */
  public void close() {
    sparkFlex.close();
  }

  /**
   * Get the firmware version of the SPARK.
   *
   * @return uint32_t Firmware version integer. Value is represented as 4 bytes, Major.Minor.Build
   *     H.Build L
   */
  public int getFirmwareVersion() {
    return sparkFlex.getFirmwareVersion();
  }

  /**
   * Set the control frame send period for the native CAN Send thread.
   *
   * @param periodMs The send period in milliseconds between 1ms and 100ms or set to 0 to disable
   *     periodic sends. Note this is not updated until the next call to Set() or SetReference().
   */
  public void setControlFramePeriodMs(int periodMs) {
    sparkFlex.setControlFramePeriodMs(periodMs);
  }

  /**
   * Get the firmware version of the SPARK as a string.
   *
   * @return std::string Human readable firmware version string
   */
  public String getFirmwareString() {
    return sparkFlex.getFirmwareString();
  }

  /**
   * Get the unique serial number of the SPARK. Not currently available.
   *
   * @return byte[] Vector of bytes representig the unique serial number
   */
  public byte[] getSerialNumber() {
    return sparkFlex.getSerialNumber();
  }

  /**
   * Get the configured Device ID of the SPARK.
   *
   * @return int device ID
   */
  public int getDeviceId() {
    return sparkFlex.getDeviceId();
  }

  /**
   * Get the motor type setting for the SPARK
   *
   * @return MotorType Motor type setting
   */
  public MotorType getMotorType() {
    return sparkFlex.getMotorType();
  }

  /**
   * Set the amount of time to wait for a periodic status frame before returning a timeout error.
   * This timeout will apply to all periodic status frames for the SPARK motor controller.
   *
   * <p>To prevent invalid timeout errors, the minimum timeout for a given periodic status is 2.1
   * times its period. To use the minimum timeout for all status frames, set timeoutMs to 0.
   *
   * <p>The default timeout is 500ms.
   *
   * @param timeoutMs The timeout in milliseconds
   */
  public void setPeriodicFrameTimeout(int timeoutMs) {
    sparkFlex.setPeriodicFrameTimeout(timeoutMs);
  }

  /**
   * Set the maximum number of times to retry an RTR CAN frame. This applies to calls such as
   * SetParameter* and GetParameter* where a request is made to the SPARK motor controller and a
   * response is expected. Anytime sending the request or receiving the response fails, it will
   * retry the request a number of times, no more than the value set by this method. If an attempt
   * succeeds, it will immediately return. The minimum number of retries is 0, where only a single
   * attempt will be made and will return regardless of success or failure.
   *
   * <p>The default maximum is 5 retries.
   *
   * @param numRetries The maximum number of retries
   */
  public void setCANMaxRetries(int numRetries) {
    sparkFlex.setCANMaxRetries(numRetries);
  }

  public float getSafeFloat(float f) {
    return sparkFlex.getSafeFloat(f);
  }

  /** Create the sim gui Fault Manager for this Spark Device */
  public void createSimFaultManager() {
    sparkFlex.createSimFaultManager();
  }

  public void setIdleMode(SparkBaseConfig.IdleMode idleMode) {
    config.idleMode(idleMode);
    reconfigure();
  }

  public ClosedLoopConfig getClosedLoopConfig() {
    return config.closedLoop;
  }

  public AbsoluteEncoderConfig getAbsoluteEncoderConfig() {
    return config.absoluteEncoder;
  }

  public AnalogSensorConfig getAnalogSensorConfig() {
    return config.analogSensor;
  }

  public EncoderConfig getEncoderConfig() {
    return config.encoder;
  }

  public ExternalEncoderConfig getExternalEncoderConfig() {
    return config.externalEncoder;
  }

  public LimitSwitchConfig getLimitSwitchConfig() {
    return config.limitSwitch;
  }

  public SoftLimitConfig getSoftLimitConfig() {
    return config.softLimit;
  }

  public void regenerateConfiguration() {
    reconfigure();
  }

  public void setResetMode(ResetMode resetMode) {
    this.resetMode = resetMode;
    reconfigure();
  }

  public void setPersistMode(PersistMode persistMode) {
    this.persistMode = persistMode;
    reconfigure();
  }

  /**
   * Sets the linear plant that runs this motor for simulation purposes.
   *
   * @param system the plant for the motor simulation.
   */
  public void setSimPlant(LinearSystem<N2, N1, N2> system) {
    this.plant = system;
    regenerateSimObjects();
  }

  /**
   * Sets the gearbox that runs this motor for simulation purposes.
   *
   * @param gearbox the gearbox for the motor simulation.
   */
  public void setSimGearbox(DCMotor gearbox) {
    this.gearbox = gearbox;
    regenerateConfiguration();
  }

  private void regenerateSimObjects() {
    motorSim = new DCMotorSim((plant), gearbox, 0.0);
    sparkFlexSim = new SparkFlexSim(sparkFlex, gearbox);
  }

  /**
   * Updates the state of the RSSparkFlex for simulation purposes. Make sure to call this in the
   * {@link Subsystem#simulationPeriodic()} method.
   */
  public void simUpdate() {
    if (isSimulation) {
      motorSim.setInput(sparkFlex.getAppliedOutput() * RoboRioSim.getVInVoltage());
      sparkFlexSim.iterate(motorSim.getAngularVelocityRPM(), RoboRioSim.getVInVoltage(), 0.02);
    }
  }

  @Override
  public void initSendable(SendableBuilder builder) {
    builder.addDoubleProperty("Velocity", this::getAppliedOutput, null);
    builder.addDoubleProperty("Current", this::getOutputCurrent, null);
  }
}
