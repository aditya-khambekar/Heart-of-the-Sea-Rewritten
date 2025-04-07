package org.team4639.lib.motorcontrol.talonfx;

import static edu.wpi.first.units.Units.Radians;
import static edu.wpi.first.units.Units.RadiansPerSecond;

import com.ctre.phoenix6.BaseStatusSignal;
import com.ctre.phoenix6.CANBus;
import com.ctre.phoenix6.StatusCode;
import com.ctre.phoenix6.StatusSignal;
import com.ctre.phoenix6.configs.MotorOutputConfigs;
import com.ctre.phoenix6.configs.TalonFXConfigurator;
import com.ctre.phoenix6.controls.*;
import com.ctre.phoenix6.controls.compound.*;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.*;
import com.ctre.phoenix6.sim.TalonFXSimState;
import edu.wpi.first.math.numbers.N1;
import edu.wpi.first.math.numbers.N2;
import edu.wpi.first.math.system.LinearSystem;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.math.system.plant.LinearSystemId;
import edu.wpi.first.units.AngularVelocityUnit;
import edu.wpi.first.units.CurrentUnit;
import edu.wpi.first.units.TorqueUnit;
import edu.wpi.first.units.VoltageUnit;
import edu.wpi.first.units.measure.*;
import edu.wpi.first.util.sendable.Sendable;
import edu.wpi.first.util.sendable.SendableBuilder;
import edu.wpi.first.wpilibj.simulation.DCMotorSim;
import java.util.function.BooleanSupplier;
import org.littletonrobotics.junction.AutoLogOutput;
import org.team4639.lib.motorcontrol.generic.NeutralMode;
import org.team4639.lib.motorcontrol.generic.RSGenericMotorController;

public class RSTalonFX implements RSGenericMotorController, Sendable {
  protected TalonFX talonFX;
  protected DCMotorSim simMotor;

  public void setGearbox(DCMotor gearbox) {
    this.gearbox = gearbox;
    regenerateSim();
  }

  public void setPlant(LinearSystem<N2, N1, N2> plant) {
    this.plant = plant;
    regenerateSim();
  }

  protected DCMotor gearbox;
  protected LinearSystem<N2, N1, N2> plant;

  protected boolean inverted = false;
  protected String name = "RSTalonFX" + this;

  private void init() {
    gearbox = DCMotor.getKrakenX60(1);
    plant = LinearSystemId.createDCMotorSystem(gearbox, 0, 1.0);
    simMotor = new DCMotorSim(plant, gearbox, 0.0);

    boolean inverted = false;
    String name = "RSTalonFX" + this;
  }

  private void regenerateSim() {
    simMotor = new DCMotorSim(plant, gearbox, 0.0);
  }

  /**
   * Constructs a new Talon FX motor controller object.
   *
   * <p>Constructs the device using the default CAN bus for the system:
   *
   * <ul>
   *   <li>"rio" on roboRIO
   *   <li>"can0" on Linux
   *   <li>"*" on Windows
   * </ul>
   *
   * @param deviceId ID of the device, as configured in Phoenix Tuner.
   */
  public RSTalonFX(int deviceId) {
    talonFX = new TalonFX(deviceId);
    init();
  }

  /**
   * Constructs a new Talon FX motor controller object.
   *
   * @param deviceId ID of the device, as configured in Phoenix Tuner.
   * @param canbus The CAN bus this device is on.
   */
  public RSTalonFX(int deviceId, CANBus canbus) {
    talonFX = new TalonFX(deviceId, canbus);
    init();
  }

  public void setName(String name) {
    this.name = name;
  }

  public RSTalonFX withName(String name) {
    this.name = name;
    return this;
  }

  /**
   * Gets the configurator to use with this device's configs
   *
   * @return Configurator for this object
   */
  public TalonFXConfigurator getConfigurator() {
    return talonFX.getConfigurator();
  }

  /**
   * Get the simulation state for this device.
   *
   * <p>This function reuses an allocated simulation state object, so it is safe to call this
   * function multiple times in a robot loop.
   *
   * @return Simulation state
   */
  public TalonFXSimState getSimState() {
    return talonFX.getSimState();
  }

  /**
   * App Major Version number.
   *
   * <ul>
   *   <li><b>Minimum Value:</b> 0
   *   <li><b>Maximum Value:</b> 255
   *   <li><b>Default Value:</b> 0
   *   <li><b>Units:</b>
   * </ul>
   *
   * <p>Default Rates:
   *
   * <ul>
   *   <li><b>CAN:</b> 4.0 Hz
   * </ul>
   *
   * <p>This refreshes and returns a cached StatusSignal object.
   *
   * @return VersionMajor Status Signal Object
   */
  public StatusSignal<Integer> getVersionMajor() {
    return talonFX.getVersionMajor();
  }

  /**
   * App Major Version number.
   *
   * <ul>
   *   <li><b>Minimum Value:</b> 0
   *   <li><b>Maximum Value:</b> 255
   *   <li><b>Default Value:</b> 0
   *   <li><b>Units:</b>
   * </ul>
   *
   * <p>Default Rates:
   *
   * <ul>
   *   <li><b>CAN:</b> 4.0 Hz
   * </ul>
   *
   * <p>This refreshes and returns a cached StatusSignal object.
   *
   * @param refresh Whether to refresh the StatusSignal before returning it; defaults to true
   * @return VersionMajor Status Signal Object
   */
  public StatusSignal<Integer> getVersionMajor(boolean refresh) {
    return talonFX.getVersionMajor(refresh);
  }

  /**
   * App Minor Version number.
   *
   * <ul>
   *   <li><b>Minimum Value:</b> 0
   *   <li><b>Maximum Value:</b> 255
   *   <li><b>Default Value:</b> 0
   *   <li><b>Units:</b>
   * </ul>
   *
   * <p>Default Rates:
   *
   * <ul>
   *   <li><b>CAN:</b> 4.0 Hz
   * </ul>
   *
   * <p>This refreshes and returns a cached StatusSignal object.
   *
   * @return VersionMinor Status Signal Object
   */
  public StatusSignal<Integer> getVersionMinor() {
    return talonFX.getVersionMinor();
  }

  /**
   * App Minor Version number.
   *
   * <ul>
   *   <li><b>Minimum Value:</b> 0
   *   <li><b>Maximum Value:</b> 255
   *   <li><b>Default Value:</b> 0
   *   <li><b>Units:</b>
   * </ul>
   *
   * <p>Default Rates:
   *
   * <ul>
   *   <li><b>CAN:</b> 4.0 Hz
   * </ul>
   *
   * <p>This refreshes and returns a cached StatusSignal object.
   *
   * @param refresh Whether to refresh the StatusSignal before returning it; defaults to true
   * @return VersionMinor Status Signal Object
   */
  public StatusSignal<Integer> getVersionMinor(boolean refresh) {
    return talonFX.getVersionMinor(refresh);
  }

  /**
   * App Bugfix Version number.
   *
   * <ul>
   *   <li><b>Minimum Value:</b> 0
   *   <li><b>Maximum Value:</b> 255
   *   <li><b>Default Value:</b> 0
   *   <li><b>Units:</b>
   * </ul>
   *
   * <p>Default Rates:
   *
   * <ul>
   *   <li><b>CAN:</b> 4.0 Hz
   * </ul>
   *
   * <p>This refreshes and returns a cached StatusSignal object.
   *
   * @return VersionBugfix Status Signal Object
   */
  public StatusSignal<Integer> getVersionBugfix() {
    return talonFX.getVersionBugfix();
  }

  /**
   * App Bugfix Version number.
   *
   * <ul>
   *   <li><b>Minimum Value:</b> 0
   *   <li><b>Maximum Value:</b> 255
   *   <li><b>Default Value:</b> 0
   *   <li><b>Units:</b>
   * </ul>
   *
   * <p>Default Rates:
   *
   * <ul>
   *   <li><b>CAN:</b> 4.0 Hz
   * </ul>
   *
   * <p>This refreshes and returns a cached StatusSignal object.
   *
   * @param refresh Whether to refresh the StatusSignal before returning it; defaults to true
   * @return VersionBugfix Status Signal Object
   */
  public StatusSignal<Integer> getVersionBugfix(boolean refresh) {
    return talonFX.getVersionBugfix(refresh);
  }

  /**
   * App Build Version number.
   *
   * <ul>
   *   <li><b>Minimum Value:</b> 0
   *   <li><b>Maximum Value:</b> 255
   *   <li><b>Default Value:</b> 0
   *   <li><b>Units:</b>
   * </ul>
   *
   * <p>Default Rates:
   *
   * <ul>
   *   <li><b>CAN:</b> 4.0 Hz
   * </ul>
   *
   * <p>This refreshes and returns a cached StatusSignal object.
   *
   * @return VersionBuild Status Signal Object
   */
  public StatusSignal<Integer> getVersionBuild() {
    return talonFX.getVersionBuild();
  }

  /**
   * App Build Version number.
   *
   * <ul>
   *   <li><b>Minimum Value:</b> 0
   *   <li><b>Maximum Value:</b> 255
   *   <li><b>Default Value:</b> 0
   *   <li><b>Units:</b>
   * </ul>
   *
   * <p>Default Rates:
   *
   * <ul>
   *   <li><b>CAN:</b> 4.0 Hz
   * </ul>
   *
   * <p>This refreshes and returns a cached StatusSignal object.
   *
   * @param refresh Whether to refresh the StatusSignal before returning it; defaults to true
   * @return VersionBuild Status Signal Object
   */
  public StatusSignal<Integer> getVersionBuild(boolean refresh) {
    return talonFX.getVersionBuild(refresh);
  }

  /**
   * Full Version of firmware in device. The format is a four byte value.
   *
   * <ul>
   *   <li><b>Minimum Value:</b> 0
   *   <li><b>Maximum Value:</b> 4294967295
   *   <li><b>Default Value:</b> 0
   *   <li><b>Units:</b>
   * </ul>
   *
   * <p>Default Rates:
   *
   * <ul>
   *   <li><b>CAN:</b> 4.0 Hz
   * </ul>
   *
   * <p>This refreshes and returns a cached StatusSignal object.
   *
   * @return Version Status Signal Object
   */
  public StatusSignal<Integer> getVersion() {
    return talonFX.getVersion();
  }

  /**
   * Full Version of firmware in device. The format is a four byte value.
   *
   * <ul>
   *   <li><b>Minimum Value:</b> 0
   *   <li><b>Maximum Value:</b> 4294967295
   *   <li><b>Default Value:</b> 0
   *   <li><b>Units:</b>
   * </ul>
   *
   * <p>Default Rates:
   *
   * <ul>
   *   <li><b>CAN:</b> 4.0 Hz
   * </ul>
   *
   * <p>This refreshes and returns a cached StatusSignal object.
   *
   * @param refresh Whether to refresh the StatusSignal before returning it; defaults to true
   * @return Version Status Signal Object
   */
  public StatusSignal<Integer> getVersion(boolean refresh) {
    return talonFX.getVersion(refresh);
  }

  /**
   * Integer representing all fault flags reported by the device.
   *
   * <p>These are device specific and are not used directly in typical applications. Use the signal
   * specific GetFault_*() methods instead.
   *
   * <ul>
   *   <li><b>Minimum Value:</b> 0
   *   <li><b>Maximum Value:</b> 4294967295
   *   <li><b>Default Value:</b> 0
   *   <li><b>Units:</b>
   * </ul>
   *
   * <p>Default Rates:
   *
   * <ul>
   *   <li><b>CAN:</b> 4.0 Hz
   * </ul>
   *
   * <p>This refreshes and returns a cached StatusSignal object.
   *
   * @return FaultField Status Signal Object
   */
  public StatusSignal<Integer> getFaultField() {
    return talonFX.getFaultField();
  }

  /**
   * Integer representing all fault flags reported by the device.
   *
   * <p>These are device specific and are not used directly in typical applications. Use the signal
   * specific GetFault_*() methods instead.
   *
   * <ul>
   *   <li><b>Minimum Value:</b> 0
   *   <li><b>Maximum Value:</b> 4294967295
   *   <li><b>Default Value:</b> 0
   *   <li><b>Units:</b>
   * </ul>
   *
   * <p>Default Rates:
   *
   * <ul>
   *   <li><b>CAN:</b> 4.0 Hz
   * </ul>
   *
   * <p>This refreshes and returns a cached StatusSignal object.
   *
   * @param refresh Whether to refresh the StatusSignal before returning it; defaults to true
   * @return FaultField Status Signal Object
   */
  public StatusSignal<Integer> getFaultField(boolean refresh) {
    return talonFX.getFaultField(refresh);
  }

  /**
   * Integer representing all (persistent) sticky fault flags reported by the device.
   *
   * <p>These are device specific and are not used directly in typical applications. Use the signal
   * specific GetStickyFault_*() methods instead.
   *
   * <ul>
   *   <li><b>Minimum Value:</b> 0
   *   <li><b>Maximum Value:</b> 4294967295
   *   <li><b>Default Value:</b> 0
   *   <li><b>Units:</b>
   * </ul>
   *
   * <p>Default Rates:
   *
   * <ul>
   *   <li><b>CAN:</b> 4.0 Hz
   * </ul>
   *
   * <p>This refreshes and returns a cached StatusSignal object.
   *
   * @return StickyFaultField Status Signal Object
   */
  public StatusSignal<Integer> getStickyFaultField() {
    return talonFX.getStickyFaultField();
  }

  /**
   * Integer representing all (persistent) sticky fault flags reported by the device.
   *
   * <p>These are device specific and are not used directly in typical applications. Use the signal
   * specific GetStickyFault_*() methods instead.
   *
   * <ul>
   *   <li><b>Minimum Value:</b> 0
   *   <li><b>Maximum Value:</b> 4294967295
   *   <li><b>Default Value:</b> 0
   *   <li><b>Units:</b>
   * </ul>
   *
   * <p>Default Rates:
   *
   * <ul>
   *   <li><b>CAN:</b> 4.0 Hz
   * </ul>
   *
   * <p>This refreshes and returns a cached StatusSignal object.
   *
   * @param refresh Whether to refresh the StatusSignal before returning it; defaults to true
   * @return StickyFaultField Status Signal Object
   */
  public StatusSignal<Integer> getStickyFaultField(boolean refresh) {
    return talonFX.getStickyFaultField(refresh);
  }

  /**
   * The applied (output) motor voltage.
   *
   * <ul>
   *   <li><b>Minimum Value:</b> -40.96
   *   <li><b>Maximum Value:</b> 40.95
   *   <li><b>Default Value:</b> 0
   *   <li><b>Units:</b> V
   * </ul>
   *
   * <p>Default Rates:
   *
   * <ul>
   *   <li><b>CAN 2.0:</b> 100.0 Hz
   *   <li><b>CAN FD:</b> 100.0 Hz (TimeSynced with Pro)
   * </ul>
   *
   * <p>This refreshes and returns a cached StatusSignal object.
   *
   * @return MotorVoltage Status Signal Object
   */
  public StatusSignal<Voltage> getMotorVoltage() {
    return talonFX.getMotorVoltage();
  }

  /**
   * The applied (output) motor voltage.
   *
   * <ul>
   *   <li><b>Minimum Value:</b> -40.96
   *   <li><b>Maximum Value:</b> 40.95
   *   <li><b>Default Value:</b> 0
   *   <li><b>Units:</b> V
   * </ul>
   *
   * <p>Default Rates:
   *
   * <ul>
   *   <li><b>CAN 2.0:</b> 100.0 Hz
   *   <li><b>CAN FD:</b> 100.0 Hz (TimeSynced with Pro)
   * </ul>
   *
   * <p>This refreshes and returns a cached StatusSignal object.
   *
   * @param refresh Whether to refresh the StatusSignal before returning it; defaults to true
   * @return MotorVoltage Status Signal Object
   */
  public StatusSignal<Voltage> getMotorVoltage(boolean refresh) {
    return talonFX.getMotorVoltage(refresh);
  }

  /**
   * Forward Limit Pin.
   *
   * <p>
   *
   * <p>Default Rates:
   *
   * <ul>
   *   <li><b>CAN 2.0:</b> 100.0 Hz
   *   <li><b>CAN FD:</b> 100.0 Hz (TimeSynced with Pro)
   * </ul>
   *
   * <p>This refreshes and returns a cached StatusSignal object.
   *
   * @return ForwardLimit Status Signal Object
   */
  public StatusSignal<ForwardLimitValue> getForwardLimit() {
    return talonFX.getForwardLimit();
  }

  /**
   * Forward Limit Pin.
   *
   * <p>
   *
   * <p>Default Rates:
   *
   * <ul>
   *   <li><b>CAN 2.0:</b> 100.0 Hz
   *   <li><b>CAN FD:</b> 100.0 Hz (TimeSynced with Pro)
   * </ul>
   *
   * <p>This refreshes and returns a cached StatusSignal object.
   *
   * @param refresh Whether to refresh the StatusSignal before returning it; defaults to true
   * @return ForwardLimit Status Signal Object
   */
  public StatusSignal<ForwardLimitValue> getForwardLimit(boolean refresh) {
    return talonFX.getForwardLimit(refresh);
  }

  /**
   * Reverse Limit Pin.
   *
   * <p>
   *
   * <p>Default Rates:
   *
   * <ul>
   *   <li><b>CAN 2.0:</b> 100.0 Hz
   *   <li><b>CAN FD:</b> 100.0 Hz (TimeSynced with Pro)
   * </ul>
   *
   * <p>This refreshes and returns a cached StatusSignal object.
   *
   * @return ReverseLimit Status Signal Object
   */
  public StatusSignal<ReverseLimitValue> getReverseLimit() {
    return talonFX.getReverseLimit();
  }

  /**
   * Reverse Limit Pin.
   *
   * <p>
   *
   * <p>Default Rates:
   *
   * <ul>
   *   <li><b>CAN 2.0:</b> 100.0 Hz
   *   <li><b>CAN FD:</b> 100.0 Hz (TimeSynced with Pro)
   * </ul>
   *
   * <p>This refreshes and returns a cached StatusSignal object.
   *
   * @param refresh Whether to refresh the StatusSignal before returning it; defaults to true
   * @return ReverseLimit Status Signal Object
   */
  public StatusSignal<ReverseLimitValue> getReverseLimit(boolean refresh) {
    return talonFX.getReverseLimit(refresh);
  }

  /**
   * The applied rotor polarity as seen from the front of the motor. This typically is determined by
   * the Inverted config, but can be overridden if using Follower features.
   *
   * <p>
   *
   * <p>Default Rates:
   *
   * <ul>
   *   <li><b>CAN 2.0:</b> 100.0 Hz
   *   <li><b>CAN FD:</b> 100.0 Hz (TimeSynced with Pro)
   * </ul>
   *
   * <p>This refreshes and returns a cached StatusSignal object.
   *
   * @return AppliedRotorPolarity Status Signal Object
   */
  public StatusSignal<AppliedRotorPolarityValue> getAppliedRotorPolarity() {
    return talonFX.getAppliedRotorPolarity();
  }

  /**
   * The applied rotor polarity as seen from the front of the motor. This typically is determined by
   * the Inverted config, but can be overridden if using Follower features.
   *
   * <p>
   *
   * <p>Default Rates:
   *
   * <ul>
   *   <li><b>CAN 2.0:</b> 100.0 Hz
   *   <li><b>CAN FD:</b> 100.0 Hz (TimeSynced with Pro)
   * </ul>
   *
   * <p>This refreshes and returns a cached StatusSignal object.
   *
   * @param refresh Whether to refresh the StatusSignal before returning it; defaults to true
   * @return AppliedRotorPolarity Status Signal Object
   */
  public StatusSignal<AppliedRotorPolarityValue> getAppliedRotorPolarity(boolean refresh) {
    return talonFX.getAppliedRotorPolarity(refresh);
  }

  /**
   * The applied motor duty cycle.
   *
   * <ul>
   *   <li><b>Minimum Value:</b> -2.0
   *   <li><b>Maximum Value:</b> 1.9990234375
   *   <li><b>Default Value:</b> 0
   *   <li><b>Units:</b> fractional
   * </ul>
   *
   * <p>Default Rates:
   *
   * <ul>
   *   <li><b>CAN 2.0:</b> 100.0 Hz
   *   <li><b>CAN FD:</b> 100.0 Hz (TimeSynced with Pro)
   * </ul>
   *
   * <p>This refreshes and returns a cached StatusSignal object.
   *
   * @return DutyCycle Status Signal Object
   */
  public StatusSignal<Double> getDutyCycle() {
    return talonFX.getDutyCycle();
  }

  /**
   * The applied motor duty cycle.
   *
   * <ul>
   *   <li><b>Minimum Value:</b> -2.0
   *   <li><b>Maximum Value:</b> 1.9990234375
   *   <li><b>Default Value:</b> 0
   *   <li><b>Units:</b> fractional
   * </ul>
   *
   * <p>Default Rates:
   *
   * <ul>
   *   <li><b>CAN 2.0:</b> 100.0 Hz
   *   <li><b>CAN FD:</b> 100.0 Hz (TimeSynced with Pro)
   * </ul>
   *
   * <p>This refreshes and returns a cached StatusSignal object.
   *
   * @param refresh Whether to refresh the StatusSignal before returning it; defaults to true
   * @return DutyCycle Status Signal Object
   */
  public StatusSignal<Double> getDutyCycle(boolean refresh) {
    return talonFX.getDutyCycle(refresh);
  }

  /**
   * Current corresponding to the torque output by the motor. Similar to StatorCurrent. Users will
   * likely prefer this current to calculate the applied torque to the rotor.
   *
   * <p>Stator current where positive current means torque is applied in the forward direction as
   * determined by the Inverted setting.
   *
   * <ul>
   *   <li><b>Minimum Value:</b> -327.68
   *   <li><b>Maximum Value:</b> 327.67
   *   <li><b>Default Value:</b> 0
   *   <li><b>Units:</b> A
   * </ul>
   *
   * <p>Default Rates:
   *
   * <ul>
   *   <li><b>CAN 2.0:</b> 100.0 Hz
   *   <li><b>CAN FD:</b> 100.0 Hz (TimeSynced with Pro)
   * </ul>
   *
   * <p>This refreshes and returns a cached StatusSignal object.
   *
   * @return TorqueCurrent Status Signal Object
   */
  public StatusSignal<Current> getTorqueCurrent() {
    return talonFX.getTorqueCurrent();
  }

  /**
   * Current corresponding to the torque output by the motor. Similar to StatorCurrent. Users will
   * likely prefer this current to calculate the applied torque to the rotor.
   *
   * <p>Stator current where positive current means torque is applied in the forward direction as
   * determined by the Inverted setting.
   *
   * <ul>
   *   <li><b>Minimum Value:</b> -327.68
   *   <li><b>Maximum Value:</b> 327.67
   *   <li><b>Default Value:</b> 0
   *   <li><b>Units:</b> A
   * </ul>
   *
   * <p>Default Rates:
   *
   * <ul>
   *   <li><b>CAN 2.0:</b> 100.0 Hz
   *   <li><b>CAN FD:</b> 100.0 Hz (TimeSynced with Pro)
   * </ul>
   *
   * <p>This refreshes and returns a cached StatusSignal object.
   *
   * @param refresh Whether to refresh the StatusSignal before returning it; defaults to true
   * @return TorqueCurrent Status Signal Object
   */
  public StatusSignal<Current> getTorqueCurrent(boolean refresh) {
    return talonFX.getTorqueCurrent(refresh);
  }

  /**
   * Current corresponding to the stator windings. Similar to TorqueCurrent. Users will likely
   * prefer TorqueCurrent over StatorCurrent.
   *
   * <p>Stator current where Positive current indicates motoring regardless of direction. Negative
   * current indicates regenerative braking regardless of direction.
   *
   * <ul>
   *   <li><b>Minimum Value:</b> -327.68
   *   <li><b>Maximum Value:</b> 327.66
   *   <li><b>Default Value:</b> 0
   *   <li><b>Units:</b> A
   * </ul>
   *
   * <p>Default Rates:
   *
   * <ul>
   *   <li><b>CAN 2.0:</b> 4.0 Hz
   *   <li><b>CAN FD:</b> 100.0 Hz (TimeSynced with Pro)
   * </ul>
   *
   * <p>This refreshes and returns a cached StatusSignal object.
   *
   * @return StatorCurrent Status Signal Object
   */
  public StatusSignal<Current> getStatorCurrent() {
    return talonFX.getStatorCurrent();
  }

  /**
   * Current corresponding to the stator windings. Similar to TorqueCurrent. Users will likely
   * prefer TorqueCurrent over StatorCurrent.
   *
   * <p>Stator current where Positive current indicates motoring regardless of direction. Negative
   * current indicates regenerative braking regardless of direction.
   *
   * <ul>
   *   <li><b>Minimum Value:</b> -327.68
   *   <li><b>Maximum Value:</b> 327.66
   *   <li><b>Default Value:</b> 0
   *   <li><b>Units:</b> A
   * </ul>
   *
   * <p>Default Rates:
   *
   * <ul>
   *   <li><b>CAN 2.0:</b> 4.0 Hz
   *   <li><b>CAN FD:</b> 100.0 Hz (TimeSynced with Pro)
   * </ul>
   *
   * <p>This refreshes and returns a cached StatusSignal object.
   *
   * @param refresh Whether to refresh the StatusSignal before returning it; defaults to true
   * @return StatorCurrent Status Signal Object
   */
  public StatusSignal<Current> getStatorCurrent(boolean refresh) {
    return talonFX.getStatorCurrent(refresh);
  }

  /**
   * Measured supply side current.
   *
   * <ul>
   *   <li><b>Minimum Value:</b> -327.68
   *   <li><b>Maximum Value:</b> 327.66
   *   <li><b>Default Value:</b> 0
   *   <li><b>Units:</b> A
   * </ul>
   *
   * <p>Default Rates:
   *
   * <ul>
   *   <li><b>CAN 2.0:</b> 4.0 Hz
   *   <li><b>CAN FD:</b> 100.0 Hz (TimeSynced with Pro)
   * </ul>
   *
   * <p>This refreshes and returns a cached StatusSignal object.
   *
   * @return SupplyCurrent Status Signal Object
   */
  public StatusSignal<Current> getSupplyCurrent() {
    return talonFX.getSupplyCurrent();
  }

  /**
   * Measured supply side current.
   *
   * <ul>
   *   <li><b>Minimum Value:</b> -327.68
   *   <li><b>Maximum Value:</b> 327.66
   *   <li><b>Default Value:</b> 0
   *   <li><b>Units:</b> A
   * </ul>
   *
   * <p>Default Rates:
   *
   * <ul>
   *   <li><b>CAN 2.0:</b> 4.0 Hz
   *   <li><b>CAN FD:</b> 100.0 Hz (TimeSynced with Pro)
   * </ul>
   *
   * <p>This refreshes and returns a cached StatusSignal object.
   *
   * @param refresh Whether to refresh the StatusSignal before returning it; defaults to true
   * @return SupplyCurrent Status Signal Object
   */
  public StatusSignal<Current> getSupplyCurrent(boolean refresh) {
    return talonFX.getSupplyCurrent(refresh);
  }

  /**
   * Measured supply voltage to the device.
   *
   * <ul>
   *   <li><b>Minimum Value:</b> 4
   *   <li><b>Maximum Value:</b> 29.575
   *   <li><b>Default Value:</b> 4
   *   <li><b>Units:</b> V
   * </ul>
   *
   * <p>Default Rates:
   *
   * <ul>
   *   <li><b>CAN 2.0:</b> 4.0 Hz
   *   <li><b>CAN FD:</b> 100.0 Hz (TimeSynced with Pro)
   * </ul>
   *
   * <p>This refreshes and returns a cached StatusSignal object.
   *
   * @return SupplyVoltage Status Signal Object
   */
  public StatusSignal<Voltage> getSupplyVoltage() {
    return talonFX.getSupplyVoltage();
  }

  /**
   * Measured supply voltage to the device.
   *
   * <ul>
   *   <li><b>Minimum Value:</b> 4
   *   <li><b>Maximum Value:</b> 29.575
   *   <li><b>Default Value:</b> 4
   *   <li><b>Units:</b> V
   * </ul>
   *
   * <p>Default Rates:
   *
   * <ul>
   *   <li><b>CAN 2.0:</b> 4.0 Hz
   *   <li><b>CAN FD:</b> 100.0 Hz (TimeSynced with Pro)
   * </ul>
   *
   * <p>This refreshes and returns a cached StatusSignal object.
   *
   * @param refresh Whether to refresh the StatusSignal before returning it; defaults to true
   * @return SupplyVoltage Status Signal Object
   */
  public StatusSignal<Voltage> getSupplyVoltage(boolean refresh) {
    return talonFX.getSupplyVoltage(refresh);
  }

  /**
   * Temperature of device.
   *
   * <p>This is the temperature that the device measures itself to be at. Similar to Processor
   * Temperature.
   *
   * <ul>
   *   <li><b>Minimum Value:</b> 0.0
   *   <li><b>Maximum Value:</b> 255.0
   *   <li><b>Default Value:</b> 0
   *   <li><b>Units:</b> ℃
   * </ul>
   *
   * <p>Default Rates:
   *
   * <ul>
   *   <li><b>CAN 2.0:</b> 4.0 Hz
   *   <li><b>CAN FD:</b> 100.0 Hz (TimeSynced with Pro)
   * </ul>
   *
   * <p>This refreshes and returns a cached StatusSignal object.
   *
   * @return DeviceTemp Status Signal Object
   */
  public StatusSignal<Temperature> getDeviceTemp() {
    return talonFX.getDeviceTemp();
  }

  /**
   * Temperature of device.
   *
   * <p>This is the temperature that the device measures itself to be at. Similar to Processor
   * Temperature.
   *
   * <ul>
   *   <li><b>Minimum Value:</b> 0.0
   *   <li><b>Maximum Value:</b> 255.0
   *   <li><b>Default Value:</b> 0
   *   <li><b>Units:</b> ℃
   * </ul>
   *
   * <p>Default Rates:
   *
   * <ul>
   *   <li><b>CAN 2.0:</b> 4.0 Hz
   *   <li><b>CAN FD:</b> 100.0 Hz (TimeSynced with Pro)
   * </ul>
   *
   * <p>This refreshes and returns a cached StatusSignal object.
   *
   * @param refresh Whether to refresh the StatusSignal before returning it; defaults to true
   * @return DeviceTemp Status Signal Object
   */
  public StatusSignal<Temperature> getDeviceTemp(boolean refresh) {
    return talonFX.getDeviceTemp(refresh);
  }

  /**
   * Temperature of the processor.
   *
   * <p>This is the temperature that the processor measures itself to be at. Similar to Device
   * Temperature.
   *
   * <ul>
   *   <li><b>Minimum Value:</b> 0.0
   *   <li><b>Maximum Value:</b> 255.0
   *   <li><b>Default Value:</b> 0
   *   <li><b>Units:</b> ℃
   * </ul>
   *
   * <p>Default Rates:
   *
   * <ul>
   *   <li><b>CAN 2.0:</b> 4.0 Hz
   *   <li><b>CAN FD:</b> 100.0 Hz (TimeSynced with Pro)
   * </ul>
   *
   * <p>This refreshes and returns a cached StatusSignal object.
   *
   * @return ProcessorTemp Status Signal Object
   */
  public StatusSignal<Temperature> getProcessorTemp() {
    return talonFX.getProcessorTemp();
  }

  /**
   * Temperature of the processor.
   *
   * <p>This is the temperature that the processor measures itself to be at. Similar to Device
   * Temperature.
   *
   * <ul>
   *   <li><b>Minimum Value:</b> 0.0
   *   <li><b>Maximum Value:</b> 255.0
   *   <li><b>Default Value:</b> 0
   *   <li><b>Units:</b> ℃
   * </ul>
   *
   * <p>Default Rates:
   *
   * <ul>
   *   <li><b>CAN 2.0:</b> 4.0 Hz
   *   <li><b>CAN FD:</b> 100.0 Hz (TimeSynced with Pro)
   * </ul>
   *
   * <p>This refreshes and returns a cached StatusSignal object.
   *
   * @param refresh Whether to refresh the StatusSignal before returning it; defaults to true
   * @return ProcessorTemp Status Signal Object
   */
  public StatusSignal<Temperature> getProcessorTemp(boolean refresh) {
    return talonFX.getProcessorTemp(refresh);
  }

  /**
   * Velocity of the motor rotor. This velocity is not affected by any feedback configs.
   *
   * <ul>
   *   <li><b>Minimum Value:</b> -512.0
   *   <li><b>Maximum Value:</b> 511.998046875
   *   <li><b>Default Value:</b> 0
   *   <li><b>Units:</b> rotations per second
   * </ul>
   *
   * <p>Default Rates:
   *
   * <ul>
   *   <li><b>CAN 2.0:</b> 4.0 Hz
   *   <li><b>CAN FD:</b> 100.0 Hz (TimeSynced with Pro)
   * </ul>
   *
   * <p>This refreshes and returns a cached StatusSignal object.
   *
   * @return RotorVelocity Status Signal Object
   */
  public StatusSignal<AngularVelocity> getRotorVelocity() {
    return talonFX.getRotorVelocity();
  }

  /**
   * Velocity of the motor rotor. This velocity is not affected by any feedback configs.
   *
   * <ul>
   *   <li><b>Minimum Value:</b> -512.0
   *   <li><b>Maximum Value:</b> 511.998046875
   *   <li><b>Default Value:</b> 0
   *   <li><b>Units:</b> rotations per second
   * </ul>
   *
   * <p>Default Rates:
   *
   * <ul>
   *   <li><b>CAN 2.0:</b> 4.0 Hz
   *   <li><b>CAN FD:</b> 100.0 Hz (TimeSynced with Pro)
   * </ul>
   *
   * <p>This refreshes and returns a cached StatusSignal object.
   *
   * @param refresh Whether to refresh the StatusSignal before returning it; defaults to true
   * @return RotorVelocity Status Signal Object
   */
  public StatusSignal<AngularVelocity> getRotorVelocity(boolean refresh) {
    return talonFX.getRotorVelocity(refresh);
  }

  /**
   * Position of the motor rotor. This position is only affected by the RotorOffset config and calls
   * to setPosition.
   *
   * <ul>
   *   <li><b>Minimum Value:</b> -16384.0
   *   <li><b>Maximum Value:</b> 16383.999755859375
   *   <li><b>Default Value:</b> 0
   *   <li><b>Units:</b> rotations
   * </ul>
   *
   * <p>Default Rates:
   *
   * <ul>
   *   <li><b>CAN 2.0:</b> 4.0 Hz
   *   <li><b>CAN FD:</b> 100.0 Hz (TimeSynced with Pro)
   * </ul>
   *
   * <p>This refreshes and returns a cached StatusSignal object.
   *
   * @return RotorPosition Status Signal Object
   */
  public StatusSignal<Angle> getRotorPosition() {
    return talonFX.getRotorPosition();
  }

  /**
   * Position of the motor rotor. This position is only affected by the RotorOffset config and calls
   * to setPosition.
   *
   * <ul>
   *   <li><b>Minimum Value:</b> -16384.0
   *   <li><b>Maximum Value:</b> 16383.999755859375
   *   <li><b>Default Value:</b> 0
   *   <li><b>Units:</b> rotations
   * </ul>
   *
   * <p>Default Rates:
   *
   * <ul>
   *   <li><b>CAN 2.0:</b> 4.0 Hz
   *   <li><b>CAN FD:</b> 100.0 Hz (TimeSynced with Pro)
   * </ul>
   *
   * <p>This refreshes and returns a cached StatusSignal object.
   *
   * @param refresh Whether to refresh the StatusSignal before returning it; defaults to true
   * @return RotorPosition Status Signal Object
   */
  public StatusSignal<Angle> getRotorPosition(boolean refresh) {
    return talonFX.getRotorPosition(refresh);
  }

  /**
   * Velocity of the device in mechanism rotations per second. This can be the velocity of a remote
   * sensor and is affected by the RotorToSensorRatio and SensorToMechanismRatio configs.
   *
   * <ul>
   *   <li><b>Minimum Value:</b> -512.0
   *   <li><b>Maximum Value:</b> 511.998046875
   *   <li><b>Default Value:</b> 0
   *   <li><b>Units:</b> rotations per second
   * </ul>
   *
   * <p>Default Rates:
   *
   * <ul>
   *   <li><b>CAN 2.0:</b> 50.0 Hz
   *   <li><b>CAN FD:</b> 100.0 Hz (TimeSynced with Pro)
   * </ul>
   *
   * <p>This refreshes and returns a cached StatusSignal object.
   *
   * @return Velocity Status Signal Object
   */
  @AutoLogOutput(key = "{name}velocityRPS")
  public StatusSignal<AngularVelocity> getVelocity() {
    return talonFX.getVelocity();
  }

  /**
   * Velocity of the device in mechanism rotations per second. This can be the velocity of a remote
   * sensor and is affected by the RotorToSensorRatio and SensorToMechanismRatio configs.
   *
   * <ul>
   *   <li><b>Minimum Value:</b> -512.0
   *   <li><b>Maximum Value:</b> 511.998046875
   *   <li><b>Default Value:</b> 0
   *   <li><b>Units:</b> rotations per second
   * </ul>
   *
   * <p>Default Rates:
   *
   * <ul>
   *   <li><b>CAN 2.0:</b> 50.0 Hz
   *   <li><b>CAN FD:</b> 100.0 Hz (TimeSynced with Pro)
   * </ul>
   *
   * <p>This refreshes and returns a cached StatusSignal object.
   *
   * @param refresh Whether to refresh the StatusSignal before returning it; defaults to true
   * @return Velocity Status Signal Object
   */
  public StatusSignal<AngularVelocity> getVelocity(boolean refresh) {
    return talonFX.getVelocity(refresh);
  }

  /**
   * Position of the device in mechanism rotations. This can be the position of a remote sensor and
   * is affected by the RotorToSensorRatio and SensorToMechanismRatio configs, as well as calls to
   * setPosition.
   *
   * <ul>
   *   <li><b>Minimum Value:</b> -16384.0
   *   <li><b>Maximum Value:</b> 16383.999755859375
   *   <li><b>Default Value:</b> 0
   *   <li><b>Units:</b> rotations
   * </ul>
   *
   * <p>Default Rates:
   *
   * <ul>
   *   <li><b>CAN 2.0:</b> 50.0 Hz
   *   <li><b>CAN FD:</b> 100.0 Hz (TimeSynced with Pro)
   * </ul>
   *
   * <p>This refreshes and returns a cached StatusSignal object.
   *
   * @return Position Status Signal Object
   */
  public StatusSignal<Angle> getPosition() {
    return talonFX.getPosition();
  }

  /**
   * Position of the device in mechanism rotations. This can be the position of a remote sensor and
   * is affected by the RotorToSensorRatio and SensorToMechanismRatio configs, as well as calls to
   * setPosition.
   *
   * <ul>
   *   <li><b>Minimum Value:</b> -16384.0
   *   <li><b>Maximum Value:</b> 16383.999755859375
   *   <li><b>Default Value:</b> 0
   *   <li><b>Units:</b> rotations
   * </ul>
   *
   * <p>Default Rates:
   *
   * <ul>
   *   <li><b>CAN 2.0:</b> 50.0 Hz
   *   <li><b>CAN FD:</b> 100.0 Hz (TimeSynced with Pro)
   * </ul>
   *
   * <p>This refreshes and returns a cached StatusSignal object.
   *
   * @param refresh Whether to refresh the StatusSignal before returning it; defaults to true
   * @return Position Status Signal Object
   */
  public StatusSignal<Angle> getPosition(boolean refresh) {
    return talonFX.getPosition(refresh);
  }

  /**
   * Acceleration of the device in mechanism rotations per second². This can be the acceleration of
   * a remote sensor and is affected by the RotorToSensorRatio and SensorToMechanismRatio configs.
   *
   * <ul>
   *   <li><b>Minimum Value:</b> -2048.0
   *   <li><b>Maximum Value:</b> 2047.75
   *   <li><b>Default Value:</b> 0
   *   <li><b>Units:</b> rotations per second²
   * </ul>
   *
   * <p>Default Rates:
   *
   * <ul>
   *   <li><b>CAN 2.0:</b> 50.0 Hz
   *   <li><b>CAN FD:</b> 100.0 Hz (TimeSynced with Pro)
   * </ul>
   *
   * <p>This refreshes and returns a cached StatusSignal object.
   *
   * @return Acceleration Status Signal Object
   */
  public StatusSignal<AngularAcceleration> getAcceleration() {
    return talonFX.getAcceleration();
  }

  /**
   * Acceleration of the device in mechanism rotations per second². This can be the acceleration of
   * a remote sensor and is affected by the RotorToSensorRatio and SensorToMechanismRatio configs.
   *
   * <ul>
   *   <li><b>Minimum Value:</b> -2048.0
   *   <li><b>Maximum Value:</b> 2047.75
   *   <li><b>Default Value:</b> 0
   *   <li><b>Units:</b> rotations per second²
   * </ul>
   *
   * <p>Default Rates:
   *
   * <ul>
   *   <li><b>CAN 2.0:</b> 50.0 Hz
   *   <li><b>CAN FD:</b> 100.0 Hz (TimeSynced with Pro)
   * </ul>
   *
   * <p>This refreshes and returns a cached StatusSignal object.
   *
   * @param refresh Whether to refresh the StatusSignal before returning it; defaults to true
   * @return Acceleration Status Signal Object
   */
  public StatusSignal<AngularAcceleration> getAcceleration(boolean refresh) {
    return talonFX.getAcceleration(refresh);
  }

  /**
   * The active control mode of the motor controller.
   *
   * <p>
   *
   * <p>Default Rates:
   *
   * <ul>
   *   <li><b>CAN 2.0:</b> 4.0 Hz
   *   <li><b>CAN FD:</b> 100.0 Hz (TimeSynced with Pro)
   * </ul>
   *
   * <p>This refreshes and returns a cached StatusSignal object.
   *
   * @return ControlMode Status Signal Object
   */
  public StatusSignal<ControlModeValue> getControlMode() {
    return talonFX.getControlMode();
  }

  /**
   * The active control mode of the motor controller.
   *
   * <p>
   *
   * <p>Default Rates:
   *
   * <ul>
   *   <li><b>CAN 2.0:</b> 4.0 Hz
   *   <li><b>CAN FD:</b> 100.0 Hz (TimeSynced with Pro)
   * </ul>
   *
   * <p>This refreshes and returns a cached StatusSignal object.
   *
   * @param refresh Whether to refresh the StatusSignal before returning it; defaults to true
   * @return ControlMode Status Signal Object
   */
  public StatusSignal<ControlModeValue> getControlMode(boolean refresh) {
    return talonFX.getControlMode(refresh);
  }

  /**
   * Check if Motion Magic® is running. This is equivalent to checking that the reported control
   * mode is a Motion Magic® based mode.
   *
   * <p>
   *
   * <p>Default Rates:
   *
   * <ul>
   *   <li><b>CAN 2.0:</b> 4.0 Hz
   *   <li><b>CAN FD:</b> 100.0 Hz (TimeSynced with Pro)
   * </ul>
   *
   * <p>This refreshes and returns a cached StatusSignal object.
   *
   * @return MotionMagicIsRunning Status Signal Object
   */
  public StatusSignal<MotionMagicIsRunningValue> getMotionMagicIsRunning() {
    return talonFX.getMotionMagicIsRunning();
  }

  /**
   * Check if Motion Magic® is running. This is equivalent to checking that the reported control
   * mode is a Motion Magic® based mode.
   *
   * <p>
   *
   * <p>Default Rates:
   *
   * <ul>
   *   <li><b>CAN 2.0:</b> 4.0 Hz
   *   <li><b>CAN FD:</b> 100.0 Hz (TimeSynced with Pro)
   * </ul>
   *
   * <p>This refreshes and returns a cached StatusSignal object.
   *
   * @param refresh Whether to refresh the StatusSignal before returning it; defaults to true
   * @return MotionMagicIsRunning Status Signal Object
   */
  public StatusSignal<MotionMagicIsRunningValue> getMotionMagicIsRunning(boolean refresh) {
    return talonFX.getMotionMagicIsRunning(refresh);
  }

  /**
   * Indicates if device is actuator enabled.
   *
   * <p>
   *
   * <p>Default Rates:
   *
   * <ul>
   *   <li><b>CAN 2.0:</b> 4.0 Hz
   *   <li><b>CAN FD:</b> 100.0 Hz (TimeSynced with Pro)
   * </ul>
   *
   * <p>This refreshes and returns a cached StatusSignal object.
   *
   * @return DeviceEnable Status Signal Object
   */
  public StatusSignal<DeviceEnableValue> getDeviceEnable() {
    return talonFX.getDeviceEnable();
  }

  /**
   * Indicates if device is actuator enabled.
   *
   * <p>
   *
   * <p>Default Rates:
   *
   * <ul>
   *   <li><b>CAN 2.0:</b> 4.0 Hz
   *   <li><b>CAN FD:</b> 100.0 Hz (TimeSynced with Pro)
   * </ul>
   *
   * <p>This refreshes and returns a cached StatusSignal object.
   *
   * @param refresh Whether to refresh the StatusSignal before returning it; defaults to true
   * @return DeviceEnable Status Signal Object
   */
  public StatusSignal<DeviceEnableValue> getDeviceEnable(boolean refresh) {
    return talonFX.getDeviceEnable(refresh);
  }

  /**
   * The slot that the closed-loop PID is using.
   *
   * <ul>
   *   <li><b>Minimum Value:</b> 0
   *   <li><b>Maximum Value:</b> 2
   *   <li><b>Default Value:</b> 0
   *   <li><b>Units:</b>
   * </ul>
   *
   * <p>Default Rates:
   *
   * <ul>
   *   <li><b>CAN 2.0:</b> 4.0 Hz
   *   <li><b>CAN FD:</b> 100.0 Hz (TimeSynced with Pro)
   * </ul>
   *
   * <p>This refreshes and returns a cached StatusSignal object.
   *
   * @return ClosedLoopSlot Status Signal Object
   */
  public StatusSignal<Integer> getClosedLoopSlot() {
    return talonFX.getClosedLoopSlot();
  }

  /**
   * The slot that the closed-loop PID is using.
   *
   * <ul>
   *   <li><b>Minimum Value:</b> 0
   *   <li><b>Maximum Value:</b> 2
   *   <li><b>Default Value:</b> 0
   *   <li><b>Units:</b>
   * </ul>
   *
   * <p>Default Rates:
   *
   * <ul>
   *   <li><b>CAN 2.0:</b> 4.0 Hz
   *   <li><b>CAN FD:</b> 100.0 Hz (TimeSynced with Pro)
   * </ul>
   *
   * <p>This refreshes and returns a cached StatusSignal object.
   *
   * @param refresh Whether to refresh the StatusSignal before returning it; defaults to true
   * @return ClosedLoopSlot Status Signal Object
   */
  public StatusSignal<Integer> getClosedLoopSlot(boolean refresh) {
    return talonFX.getClosedLoopSlot(refresh);
  }

  /**
   * Assess the status of the motor output with respect to load and supply.
   *
   * <p>This routine can be used to determine the general status of motor commutation.
   *
   * <p>
   *
   * <p>Default Rates:
   *
   * <ul>
   *   <li><b>CAN 2.0:</b> 4.0 Hz
   *   <li><b>CAN FD:</b> 100.0 Hz (TimeSynced with Pro)
   * </ul>
   *
   * <p>This refreshes and returns a cached StatusSignal object.
   *
   * @return MotorOutputStatus Status Signal Object
   */
  public StatusSignal<MotorOutputStatusValue> getMotorOutputStatus() {
    return talonFX.getMotorOutputStatus();
  }

  /**
   * Assess the status of the motor output with respect to load and supply.
   *
   * <p>This routine can be used to determine the general status of motor commutation.
   *
   * <p>
   *
   * <p>Default Rates:
   *
   * <ul>
   *   <li><b>CAN 2.0:</b> 4.0 Hz
   *   <li><b>CAN FD:</b> 100.0 Hz (TimeSynced with Pro)
   * </ul>
   *
   * <p>This refreshes and returns a cached StatusSignal object.
   *
   * @param refresh Whether to refresh the StatusSignal before returning it; defaults to true
   * @return MotorOutputStatus Status Signal Object
   */
  public StatusSignal<MotorOutputStatusValue> getMotorOutputStatus(boolean refresh) {
    return talonFX.getMotorOutputStatus(refresh);
  }

  /**
   * The active control mode of the differential controller.
   *
   * <p>
   *
   * <p>Default Rates:
   *
   * <ul>
   *   <li><b>CAN 2.0:</b> 100.0 Hz
   *   <li><b>CAN FD:</b> 100.0 Hz (TimeSynced with Pro)
   * </ul>
   *
   * <p>This refreshes and returns a cached StatusSignal object.
   *
   * @return DifferentialControlMode Status Signal Object
   */
  public StatusSignal<DifferentialControlModeValue> getDifferentialControlMode() {
    return talonFX.getDifferentialControlMode();
  }

  /**
   * The active control mode of the differential controller.
   *
   * <p>
   *
   * <p>Default Rates:
   *
   * <ul>
   *   <li><b>CAN 2.0:</b> 100.0 Hz
   *   <li><b>CAN FD:</b> 100.0 Hz (TimeSynced with Pro)
   * </ul>
   *
   * <p>This refreshes and returns a cached StatusSignal object.
   *
   * @param refresh Whether to refresh the StatusSignal before returning it; defaults to true
   * @return DifferentialControlMode Status Signal Object
   */
  public StatusSignal<DifferentialControlModeValue> getDifferentialControlMode(boolean refresh) {
    return talonFX.getDifferentialControlMode(refresh);
  }

  /**
   * Average component of the differential velocity of device.
   *
   * <ul>
   *   <li><b>Minimum Value:</b> -512.0
   *   <li><b>Maximum Value:</b> 511.998046875
   *   <li><b>Default Value:</b> 0
   *   <li><b>Units:</b> rotations per second
   * </ul>
   *
   * <p>Default Rates:
   *
   * <ul>
   *   <li><b>CAN 2.0:</b> 4.0 Hz
   *   <li><b>CAN FD:</b> 100.0 Hz (TimeSynced with Pro)
   * </ul>
   *
   * <p>This refreshes and returns a cached StatusSignal object.
   *
   * @return DifferentialAverageVelocity Status Signal Object
   */
  public StatusSignal<AngularVelocity> getDifferentialAverageVelocity() {
    return talonFX.getDifferentialAverageVelocity();
  }

  /**
   * Average component of the differential velocity of device.
   *
   * <ul>
   *   <li><b>Minimum Value:</b> -512.0
   *   <li><b>Maximum Value:</b> 511.998046875
   *   <li><b>Default Value:</b> 0
   *   <li><b>Units:</b> rotations per second
   * </ul>
   *
   * <p>Default Rates:
   *
   * <ul>
   *   <li><b>CAN 2.0:</b> 4.0 Hz
   *   <li><b>CAN FD:</b> 100.0 Hz (TimeSynced with Pro)
   * </ul>
   *
   * <p>This refreshes and returns a cached StatusSignal object.
   *
   * @param refresh Whether to refresh the StatusSignal before returning it; defaults to true
   * @return DifferentialAverageVelocity Status Signal Object
   */
  public StatusSignal<AngularVelocity> getDifferentialAverageVelocity(boolean refresh) {
    return talonFX.getDifferentialAverageVelocity(refresh);
  }

  /**
   * Average component of the differential position of device.
   *
   * <ul>
   *   <li><b>Minimum Value:</b> -16384.0
   *   <li><b>Maximum Value:</b> 16383.999755859375
   *   <li><b>Default Value:</b> 0
   *   <li><b>Units:</b> rotations
   * </ul>
   *
   * <p>Default Rates:
   *
   * <ul>
   *   <li><b>CAN 2.0:</b> 4.0 Hz
   *   <li><b>CAN FD:</b> 100.0 Hz (TimeSynced with Pro)
   * </ul>
   *
   * <p>This refreshes and returns a cached StatusSignal object.
   *
   * @return DifferentialAveragePosition Status Signal Object
   */
  public StatusSignal<Angle> getDifferentialAveragePosition() {
    return talonFX.getDifferentialAveragePosition();
  }

  /**
   * Average component of the differential position of device.
   *
   * <ul>
   *   <li><b>Minimum Value:</b> -16384.0
   *   <li><b>Maximum Value:</b> 16383.999755859375
   *   <li><b>Default Value:</b> 0
   *   <li><b>Units:</b> rotations
   * </ul>
   *
   * <p>Default Rates:
   *
   * <ul>
   *   <li><b>CAN 2.0:</b> 4.0 Hz
   *   <li><b>CAN FD:</b> 100.0 Hz (TimeSynced with Pro)
   * </ul>
   *
   * <p>This refreshes and returns a cached StatusSignal object.
   *
   * @param refresh Whether to refresh the StatusSignal before returning it; defaults to true
   * @return DifferentialAveragePosition Status Signal Object
   */
  public StatusSignal<Angle> getDifferentialAveragePosition(boolean refresh) {
    return talonFX.getDifferentialAveragePosition(refresh);
  }

  /**
   * Difference component of the differential velocity of device.
   *
   * <ul>
   *   <li><b>Minimum Value:</b> -512.0
   *   <li><b>Maximum Value:</b> 511.998046875
   *   <li><b>Default Value:</b> 0
   *   <li><b>Units:</b> rotations per second
   * </ul>
   *
   * <p>Default Rates:
   *
   * <ul>
   *   <li><b>CAN 2.0:</b> 4.0 Hz
   *   <li><b>CAN FD:</b> 100.0 Hz (TimeSynced with Pro)
   * </ul>
   *
   * <p>This refreshes and returns a cached StatusSignal object.
   *
   * @return DifferentialDifferenceVelocity Status Signal Object
   */
  public StatusSignal<AngularVelocity> getDifferentialDifferenceVelocity() {
    return talonFX.getDifferentialDifferenceVelocity();
  }

  /**
   * Difference component of the differential velocity of device.
   *
   * <ul>
   *   <li><b>Minimum Value:</b> -512.0
   *   <li><b>Maximum Value:</b> 511.998046875
   *   <li><b>Default Value:</b> 0
   *   <li><b>Units:</b> rotations per second
   * </ul>
   *
   * <p>Default Rates:
   *
   * <ul>
   *   <li><b>CAN 2.0:</b> 4.0 Hz
   *   <li><b>CAN FD:</b> 100.0 Hz (TimeSynced with Pro)
   * </ul>
   *
   * <p>This refreshes and returns a cached StatusSignal object.
   *
   * @param refresh Whether to refresh the StatusSignal before returning it; defaults to true
   * @return DifferentialDifferenceVelocity Status Signal Object
   */
  public StatusSignal<AngularVelocity> getDifferentialDifferenceVelocity(boolean refresh) {
    return talonFX.getDifferentialDifferenceVelocity(refresh);
  }

  /**
   * Difference component of the differential position of device.
   *
   * <ul>
   *   <li><b>Minimum Value:</b> -16384.0
   *   <li><b>Maximum Value:</b> 16383.999755859375
   *   <li><b>Default Value:</b> 0
   *   <li><b>Units:</b> rotations
   * </ul>
   *
   * <p>Default Rates:
   *
   * <ul>
   *   <li><b>CAN 2.0:</b> 4.0 Hz
   *   <li><b>CAN FD:</b> 100.0 Hz (TimeSynced with Pro)
   * </ul>
   *
   * <p>This refreshes and returns a cached StatusSignal object.
   *
   * @return DifferentialDifferencePosition Status Signal Object
   */
  public StatusSignal<Angle> getDifferentialDifferencePosition() {
    return talonFX.getDifferentialDifferencePosition();
  }

  /**
   * Difference component of the differential position of device.
   *
   * <ul>
   *   <li><b>Minimum Value:</b> -16384.0
   *   <li><b>Maximum Value:</b> 16383.999755859375
   *   <li><b>Default Value:</b> 0
   *   <li><b>Units:</b> rotations
   * </ul>
   *
   * <p>Default Rates:
   *
   * <ul>
   *   <li><b>CAN 2.0:</b> 4.0 Hz
   *   <li><b>CAN FD:</b> 100.0 Hz (TimeSynced with Pro)
   * </ul>
   *
   * <p>This refreshes and returns a cached StatusSignal object.
   *
   * @param refresh Whether to refresh the StatusSignal before returning it; defaults to true
   * @return DifferentialDifferencePosition Status Signal Object
   */
  public StatusSignal<Angle> getDifferentialDifferencePosition(boolean refresh) {
    return talonFX.getDifferentialDifferencePosition(refresh);
  }

  /**
   * The slot that the closed-loop differential PID is using.
   *
   * <ul>
   *   <li><b>Minimum Value:</b> 0
   *   <li><b>Maximum Value:</b> 2
   *   <li><b>Default Value:</b> 0
   *   <li><b>Units:</b>
   * </ul>
   *
   * <p>Default Rates:
   *
   * <ul>
   *   <li><b>CAN 2.0:</b> 4.0 Hz
   *   <li><b>CAN FD:</b> 100.0 Hz (TimeSynced with Pro)
   * </ul>
   *
   * <p>This refreshes and returns a cached StatusSignal object.
   *
   * @return DifferentialClosedLoopSlot Status Signal Object
   */
  public StatusSignal<Integer> getDifferentialClosedLoopSlot() {
    return talonFX.getDifferentialClosedLoopSlot();
  }

  /**
   * The slot that the closed-loop differential PID is using.
   *
   * <ul>
   *   <li><b>Minimum Value:</b> 0
   *   <li><b>Maximum Value:</b> 2
   *   <li><b>Default Value:</b> 0
   *   <li><b>Units:</b>
   * </ul>
   *
   * <p>Default Rates:
   *
   * <ul>
   *   <li><b>CAN 2.0:</b> 4.0 Hz
   *   <li><b>CAN FD:</b> 100.0 Hz (TimeSynced with Pro)
   * </ul>
   *
   * <p>This refreshes and returns a cached StatusSignal object.
   *
   * @param refresh Whether to refresh the StatusSignal before returning it; defaults to true
   * @return DifferentialClosedLoopSlot Status Signal Object
   */
  public StatusSignal<Integer> getDifferentialClosedLoopSlot(boolean refresh) {
    return talonFX.getDifferentialClosedLoopSlot(refresh);
  }

  /**
   * The torque constant (K_T) of the motor.
   *
   * <ul>
   *   <li><b>Minimum Value:</b> 0.0
   *   <li><b>Maximum Value:</b> 0.025500000000000002
   *   <li><b>Default Value:</b> 0
   *   <li><b>Units:</b> Nm/A
   * </ul>
   *
   * <p>Default Rates:
   *
   * <ul>
   *   <li><b>CAN:</b> 4.0 Hz
   * </ul>
   *
   * <p>This refreshes and returns a cached StatusSignal object.
   *
   * @return MotorKT Status Signal Object
   */
  public StatusSignal<Per<TorqueUnit, CurrentUnit>> getMotorKT() {
    return talonFX.getMotorKT();
  }

  /**
   * The torque constant (K_T) of the motor.
   *
   * <ul>
   *   <li><b>Minimum Value:</b> 0.0
   *   <li><b>Maximum Value:</b> 0.025500000000000002
   *   <li><b>Default Value:</b> 0
   *   <li><b>Units:</b> Nm/A
   * </ul>
   *
   * <p>Default Rates:
   *
   * <ul>
   *   <li><b>CAN:</b> 4.0 Hz
   * </ul>
   *
   * <p>This refreshes and returns a cached StatusSignal object.
   *
   * @param refresh Whether to refresh the StatusSignal before returning it; defaults to true
   * @return MotorKT Status Signal Object
   */
  public StatusSignal<Per<TorqueUnit, CurrentUnit>> getMotorKT(boolean refresh) {
    return talonFX.getMotorKT(refresh);
  }

  /**
   * The velocity constant (K_V) of the motor.
   *
   * <ul>
   *   <li><b>Minimum Value:</b> 0.0
   *   <li><b>Maximum Value:</b> 2047.0
   *   <li><b>Default Value:</b> 0
   *   <li><b>Units:</b> RPM/V
   * </ul>
   *
   * <p>Default Rates:
   *
   * <ul>
   *   <li><b>CAN:</b> 4.0 Hz
   * </ul>
   *
   * <p>This refreshes and returns a cached StatusSignal object.
   *
   * @return MotorKV Status Signal Object
   */
  public StatusSignal<Per<AngularVelocityUnit, VoltageUnit>> getMotorKV() {
    return talonFX.getMotorKV();
  }

  /**
   * The velocity constant (K_V) of the motor.
   *
   * <ul>
   *   <li><b>Minimum Value:</b> 0.0
   *   <li><b>Maximum Value:</b> 2047.0
   *   <li><b>Default Value:</b> 0
   *   <li><b>Units:</b> RPM/V
   * </ul>
   *
   * <p>Default Rates:
   *
   * <ul>
   *   <li><b>CAN:</b> 4.0 Hz
   * </ul>
   *
   * <p>This refreshes and returns a cached StatusSignal object.
   *
   * @param refresh Whether to refresh the StatusSignal before returning it; defaults to true
   * @return MotorKV Status Signal Object
   */
  public StatusSignal<Per<AngularVelocityUnit, VoltageUnit>> getMotorKV(boolean refresh) {
    return talonFX.getMotorKV(refresh);
  }

  /**
   * The stall current of the motor at 12 V output.
   *
   * <ul>
   *   <li><b>Minimum Value:</b> 0.0
   *   <li><b>Maximum Value:</b> 1023.0
   *   <li><b>Default Value:</b> 0
   *   <li><b>Units:</b> A
   * </ul>
   *
   * <p>Default Rates:
   *
   * <ul>
   *   <li><b>CAN:</b> 4.0 Hz
   * </ul>
   *
   * <p>This refreshes and returns a cached StatusSignal object.
   *
   * @return MotorStallCurrent Status Signal Object
   */
  public StatusSignal<Current> getMotorStallCurrent() {
    return talonFX.getMotorStallCurrent();
  }

  /**
   * The stall current of the motor at 12 V output.
   *
   * <ul>
   *   <li><b>Minimum Value:</b> 0.0
   *   <li><b>Maximum Value:</b> 1023.0
   *   <li><b>Default Value:</b> 0
   *   <li><b>Units:</b> A
   * </ul>
   *
   * <p>Default Rates:
   *
   * <ul>
   *   <li><b>CAN:</b> 4.0 Hz
   * </ul>
   *
   * <p>This refreshes and returns a cached StatusSignal object.
   *
   * @param refresh Whether to refresh the StatusSignal before returning it; defaults to true
   * @return MotorStallCurrent Status Signal Object
   */
  public StatusSignal<Current> getMotorStallCurrent(boolean refresh) {
    return talonFX.getMotorStallCurrent(refresh);
  }

  /**
   * The applied output of the bridge.
   *
   * <p>
   *
   * <p>Default Rates:
   *
   * <ul>
   *   <li><b>CAN 2.0:</b> 100.0 Hz
   *   <li><b>CAN FD:</b> 100.0 Hz (TimeSynced with Pro)
   * </ul>
   *
   * <p>This refreshes and returns a cached StatusSignal object.
   *
   * @return BridgeOutput Status Signal Object
   */
  public StatusSignal<BridgeOutputValue> getBridgeOutput() {
    return talonFX.getBridgeOutput();
  }

  /**
   * The applied output of the bridge.
   *
   * <p>
   *
   * <p>Default Rates:
   *
   * <ul>
   *   <li><b>CAN 2.0:</b> 100.0 Hz
   *   <li><b>CAN FD:</b> 100.0 Hz (TimeSynced with Pro)
   * </ul>
   *
   * <p>This refreshes and returns a cached StatusSignal object.
   *
   * @param refresh Whether to refresh the StatusSignal before returning it; defaults to true
   * @return BridgeOutput Status Signal Object
   */
  public StatusSignal<BridgeOutputValue> getBridgeOutput(boolean refresh) {
    return talonFX.getBridgeOutput(refresh);
  }

  /**
   * Whether the device is Phoenix Pro licensed.
   *
   * <ul>
   *   <li><b>Default Value:</b> False
   * </ul>
   *
   * <p>Default Rates:
   *
   * <ul>
   *   <li><b>CAN:</b> 4.0 Hz
   * </ul>
   *
   * <p>This refreshes and returns a cached StatusSignal object.
   *
   * @return IsProLicensed Status Signal Object
   */
  public StatusSignal<Boolean> getIsProLicensed() {
    return talonFX.getIsProLicensed();
  }

  /**
   * Whether the device is Phoenix Pro licensed.
   *
   * <ul>
   *   <li><b>Default Value:</b> False
   * </ul>
   *
   * <p>Default Rates:
   *
   * <ul>
   *   <li><b>CAN:</b> 4.0 Hz
   * </ul>
   *
   * <p>This refreshes and returns a cached StatusSignal object.
   *
   * @param refresh Whether to refresh the StatusSignal before returning it; defaults to true
   * @return IsProLicensed Status Signal Object
   */
  public StatusSignal<Boolean> getIsProLicensed(boolean refresh) {
    return talonFX.getIsProLicensed(refresh);
  }

  /**
   * Temperature of device from second sensor.
   *
   * <p>Newer versions of Talon have multiple temperature measurement methods.
   *
   * <ul>
   *   <li><b>Minimum Value:</b> 0.0
   *   <li><b>Maximum Value:</b> 255.0
   *   <li><b>Default Value:</b> 0
   *   <li><b>Units:</b> ℃
   * </ul>
   *
   * <p>Default Rates:
   *
   * <ul>
   *   <li><b>CAN 2.0:</b> 4.0 Hz
   *   <li><b>CAN FD:</b> 100.0 Hz (TimeSynced with Pro)
   * </ul>
   *
   * <p>This refreshes and returns a cached StatusSignal object.
   *
   * @return AncillaryDeviceTemp Status Signal Object
   */
  public StatusSignal<Temperature> getAncillaryDeviceTemp() {
    return talonFX.getAncillaryDeviceTemp();
  }

  /**
   * Temperature of device from second sensor.
   *
   * <p>Newer versions of Talon have multiple temperature measurement methods.
   *
   * <ul>
   *   <li><b>Minimum Value:</b> 0.0
   *   <li><b>Maximum Value:</b> 255.0
   *   <li><b>Default Value:</b> 0
   *   <li><b>Units:</b> ℃
   * </ul>
   *
   * <p>Default Rates:
   *
   * <ul>
   *   <li><b>CAN 2.0:</b> 4.0 Hz
   *   <li><b>CAN FD:</b> 100.0 Hz (TimeSynced with Pro)
   * </ul>
   *
   * <p>This refreshes and returns a cached StatusSignal object.
   *
   * @param refresh Whether to refresh the StatusSignal before returning it; defaults to true
   * @return AncillaryDeviceTemp Status Signal Object
   */
  public StatusSignal<Temperature> getAncillaryDeviceTemp(boolean refresh) {
    return talonFX.getAncillaryDeviceTemp(refresh);
  }

  /**
   * The type of motor attached to the Talon.
   *
   * <p>This can be used to determine what motor is attached to the Talon FX. Return will be
   * "Unknown" if firmware is too old or device is not present.
   *
   * <p>
   *
   * <p>Default Rates:
   *
   * <ul>
   *   <li><b>CAN 2.0:</b> 4.0 Hz
   *   <li><b>CAN FD:</b> 100.0 Hz (TimeSynced with Pro)
   * </ul>
   *
   * <p>This refreshes and returns a cached StatusSignal object.
   *
   * @return ConnectedMotor Status Signal Object
   */
  public StatusSignal<ConnectedMotorValue> getConnectedMotor() {
    return talonFX.getConnectedMotor();
  }

  /**
   * The type of motor attached to the Talon.
   *
   * <p>This can be used to determine what motor is attached to the Talon FX. Return will be
   * "Unknown" if firmware is too old or device is not present.
   *
   * <p>
   *
   * <p>Default Rates:
   *
   * <ul>
   *   <li><b>CAN 2.0:</b> 4.0 Hz
   *   <li><b>CAN FD:</b> 100.0 Hz (TimeSynced with Pro)
   * </ul>
   *
   * <p>This refreshes and returns a cached StatusSignal object.
   *
   * @param refresh Whether to refresh the StatusSignal before returning it; defaults to true
   * @return ConnectedMotor Status Signal Object
   */
  public StatusSignal<ConnectedMotorValue> getConnectedMotor(boolean refresh) {
    return talonFX.getConnectedMotor(refresh);
  }

  /**
   * Hardware fault occurred
   *
   * <ul>
   *   <li><b>Default Value:</b> False
   * </ul>
   *
   * <p>Default Rates:
   *
   * <ul>
   *   <li><b>CAN:</b> 4.0 Hz
   * </ul>
   *
   * <p>This refreshes and returns a cached StatusSignal object.
   *
   * @return Fault_Hardware Status Signal Object
   */
  public StatusSignal<Boolean> getFault_Hardware() {
    return talonFX.getFault_Hardware();
  }

  /**
   * Hardware fault occurred
   *
   * <ul>
   *   <li><b>Default Value:</b> False
   * </ul>
   *
   * <p>Default Rates:
   *
   * <ul>
   *   <li><b>CAN:</b> 4.0 Hz
   * </ul>
   *
   * <p>This refreshes and returns a cached StatusSignal object.
   *
   * @param refresh Whether to refresh the StatusSignal before returning it; defaults to true
   * @return Fault_Hardware Status Signal Object
   */
  public StatusSignal<Boolean> getFault_Hardware(boolean refresh) {
    return talonFX.getFault_Hardware(refresh);
  }

  /**
   * Hardware fault occurred
   *
   * <ul>
   *   <li><b>Default Value:</b> False
   * </ul>
   *
   * <p>Default Rates:
   *
   * <ul>
   *   <li><b>CAN:</b> 4.0 Hz
   * </ul>
   *
   * <p>This refreshes and returns a cached StatusSignal object.
   *
   * @return StickyFault_Hardware Status Signal Object
   */
  public StatusSignal<Boolean> getStickyFault_Hardware() {
    return talonFX.getStickyFault_Hardware();
  }

  /**
   * Hardware fault occurred
   *
   * <ul>
   *   <li><b>Default Value:</b> False
   * </ul>
   *
   * <p>Default Rates:
   *
   * <ul>
   *   <li><b>CAN:</b> 4.0 Hz
   * </ul>
   *
   * <p>This refreshes and returns a cached StatusSignal object.
   *
   * @param refresh Whether to refresh the StatusSignal before returning it; defaults to true
   * @return StickyFault_Hardware Status Signal Object
   */
  public StatusSignal<Boolean> getStickyFault_Hardware(boolean refresh) {
    return talonFX.getStickyFault_Hardware(refresh);
  }

  /**
   * Processor temperature exceeded limit
   *
   * <ul>
   *   <li><b>Default Value:</b> False
   * </ul>
   *
   * <p>Default Rates:
   *
   * <ul>
   *   <li><b>CAN:</b> 4.0 Hz
   * </ul>
   *
   * <p>This refreshes and returns a cached StatusSignal object.
   *
   * @return Fault_ProcTemp Status Signal Object
   */
  public StatusSignal<Boolean> getFault_ProcTemp() {
    return talonFX.getFault_ProcTemp();
  }

  /**
   * Processor temperature exceeded limit
   *
   * <ul>
   *   <li><b>Default Value:</b> False
   * </ul>
   *
   * <p>Default Rates:
   *
   * <ul>
   *   <li><b>CAN:</b> 4.0 Hz
   * </ul>
   *
   * <p>This refreshes and returns a cached StatusSignal object.
   *
   * @param refresh Whether to refresh the StatusSignal before returning it; defaults to true
   * @return Fault_ProcTemp Status Signal Object
   */
  public StatusSignal<Boolean> getFault_ProcTemp(boolean refresh) {
    return talonFX.getFault_ProcTemp(refresh);
  }

  /**
   * Processor temperature exceeded limit
   *
   * <ul>
   *   <li><b>Default Value:</b> False
   * </ul>
   *
   * <p>Default Rates:
   *
   * <ul>
   *   <li><b>CAN:</b> 4.0 Hz
   * </ul>
   *
   * <p>This refreshes and returns a cached StatusSignal object.
   *
   * @return StickyFault_ProcTemp Status Signal Object
   */
  public StatusSignal<Boolean> getStickyFault_ProcTemp() {
    return talonFX.getStickyFault_ProcTemp();
  }

  /**
   * Processor temperature exceeded limit
   *
   * <ul>
   *   <li><b>Default Value:</b> False
   * </ul>
   *
   * <p>Default Rates:
   *
   * <ul>
   *   <li><b>CAN:</b> 4.0 Hz
   * </ul>
   *
   * <p>This refreshes and returns a cached StatusSignal object.
   *
   * @param refresh Whether to refresh the StatusSignal before returning it; defaults to true
   * @return StickyFault_ProcTemp Status Signal Object
   */
  public StatusSignal<Boolean> getStickyFault_ProcTemp(boolean refresh) {
    return talonFX.getStickyFault_ProcTemp(refresh);
  }

  /**
   * Device temperature exceeded limit
   *
   * <ul>
   *   <li><b>Default Value:</b> False
   * </ul>
   *
   * <p>Default Rates:
   *
   * <ul>
   *   <li><b>CAN:</b> 4.0 Hz
   * </ul>
   *
   * <p>This refreshes and returns a cached StatusSignal object.
   *
   * @return Fault_DeviceTemp Status Signal Object
   */
  public StatusSignal<Boolean> getFault_DeviceTemp() {
    return talonFX.getFault_DeviceTemp();
  }

  /**
   * Device temperature exceeded limit
   *
   * <ul>
   *   <li><b>Default Value:</b> False
   * </ul>
   *
   * <p>Default Rates:
   *
   * <ul>
   *   <li><b>CAN:</b> 4.0 Hz
   * </ul>
   *
   * <p>This refreshes and returns a cached StatusSignal object.
   *
   * @param refresh Whether to refresh the StatusSignal before returning it; defaults to true
   * @return Fault_DeviceTemp Status Signal Object
   */
  public StatusSignal<Boolean> getFault_DeviceTemp(boolean refresh) {
    return talonFX.getFault_DeviceTemp(refresh);
  }

  /**
   * Device temperature exceeded limit
   *
   * <ul>
   *   <li><b>Default Value:</b> False
   * </ul>
   *
   * <p>Default Rates:
   *
   * <ul>
   *   <li><b>CAN:</b> 4.0 Hz
   * </ul>
   *
   * <p>This refreshes and returns a cached StatusSignal object.
   *
   * @return StickyFault_DeviceTemp Status Signal Object
   */
  public StatusSignal<Boolean> getStickyFault_DeviceTemp() {
    return talonFX.getStickyFault_DeviceTemp();
  }

  /**
   * Device temperature exceeded limit
   *
   * <ul>
   *   <li><b>Default Value:</b> False
   * </ul>
   *
   * <p>Default Rates:
   *
   * <ul>
   *   <li><b>CAN:</b> 4.0 Hz
   * </ul>
   *
   * <p>This refreshes and returns a cached StatusSignal object.
   *
   * @param refresh Whether to refresh the StatusSignal before returning it; defaults to true
   * @return StickyFault_DeviceTemp Status Signal Object
   */
  public StatusSignal<Boolean> getStickyFault_DeviceTemp(boolean refresh) {
    return talonFX.getStickyFault_DeviceTemp(refresh);
  }

  /**
   * Device supply voltage dropped to near brownout levels
   *
   * <ul>
   *   <li><b>Default Value:</b> False
   * </ul>
   *
   * <p>Default Rates:
   *
   * <ul>
   *   <li><b>CAN:</b> 4.0 Hz
   * </ul>
   *
   * <p>This refreshes and returns a cached StatusSignal object.
   *
   * @return Fault_Undervoltage Status Signal Object
   */
  public StatusSignal<Boolean> getFault_Undervoltage() {
    return talonFX.getFault_Undervoltage();
  }

  /**
   * Device supply voltage dropped to near brownout levels
   *
   * <ul>
   *   <li><b>Default Value:</b> False
   * </ul>
   *
   * <p>Default Rates:
   *
   * <ul>
   *   <li><b>CAN:</b> 4.0 Hz
   * </ul>
   *
   * <p>This refreshes and returns a cached StatusSignal object.
   *
   * @param refresh Whether to refresh the StatusSignal before returning it; defaults to true
   * @return Fault_Undervoltage Status Signal Object
   */
  public StatusSignal<Boolean> getFault_Undervoltage(boolean refresh) {
    return talonFX.getFault_Undervoltage(refresh);
  }

  /**
   * Device supply voltage dropped to near brownout levels
   *
   * <ul>
   *   <li><b>Default Value:</b> False
   * </ul>
   *
   * <p>Default Rates:
   *
   * <ul>
   *   <li><b>CAN:</b> 4.0 Hz
   * </ul>
   *
   * <p>This refreshes and returns a cached StatusSignal object.
   *
   * @return StickyFault_Undervoltage Status Signal Object
   */
  public StatusSignal<Boolean> getStickyFault_Undervoltage() {
    return talonFX.getStickyFault_Undervoltage();
  }

  /**
   * Device supply voltage dropped to near brownout levels
   *
   * <ul>
   *   <li><b>Default Value:</b> False
   * </ul>
   *
   * <p>Default Rates:
   *
   * <ul>
   *   <li><b>CAN:</b> 4.0 Hz
   * </ul>
   *
   * <p>This refreshes and returns a cached StatusSignal object.
   *
   * @param refresh Whether to refresh the StatusSignal before returning it; defaults to true
   * @return StickyFault_Undervoltage Status Signal Object
   */
  public StatusSignal<Boolean> getStickyFault_Undervoltage(boolean refresh) {
    return talonFX.getStickyFault_Undervoltage(refresh);
  }

  /**
   * Device boot while detecting the enable signal
   *
   * <ul>
   *   <li><b>Default Value:</b> False
   * </ul>
   *
   * <p>Default Rates:
   *
   * <ul>
   *   <li><b>CAN:</b> 4.0 Hz
   * </ul>
   *
   * <p>This refreshes and returns a cached StatusSignal object.
   *
   * @return Fault_BootDuringEnable Status Signal Object
   */
  public StatusSignal<Boolean> getFault_BootDuringEnable() {
    return talonFX.getFault_BootDuringEnable();
  }

  /**
   * Device boot while detecting the enable signal
   *
   * <ul>
   *   <li><b>Default Value:</b> False
   * </ul>
   *
   * <p>Default Rates:
   *
   * <ul>
   *   <li><b>CAN:</b> 4.0 Hz
   * </ul>
   *
   * <p>This refreshes and returns a cached StatusSignal object.
   *
   * @param refresh Whether to refresh the StatusSignal before returning it; defaults to true
   * @return Fault_BootDuringEnable Status Signal Object
   */
  public StatusSignal<Boolean> getFault_BootDuringEnable(boolean refresh) {
    return talonFX.getFault_BootDuringEnable(refresh);
  }

  /**
   * Device boot while detecting the enable signal
   *
   * <ul>
   *   <li><b>Default Value:</b> False
   * </ul>
   *
   * <p>Default Rates:
   *
   * <ul>
   *   <li><b>CAN:</b> 4.0 Hz
   * </ul>
   *
   * <p>This refreshes and returns a cached StatusSignal object.
   *
   * @return StickyFault_BootDuringEnable Status Signal Object
   */
  public StatusSignal<Boolean> getStickyFault_BootDuringEnable() {
    return talonFX.getStickyFault_BootDuringEnable();
  }

  /**
   * Device boot while detecting the enable signal
   *
   * <ul>
   *   <li><b>Default Value:</b> False
   * </ul>
   *
   * <p>Default Rates:
   *
   * <ul>
   *   <li><b>CAN:</b> 4.0 Hz
   * </ul>
   *
   * <p>This refreshes and returns a cached StatusSignal object.
   *
   * @param refresh Whether to refresh the StatusSignal before returning it; defaults to true
   * @return StickyFault_BootDuringEnable Status Signal Object
   */
  public StatusSignal<Boolean> getStickyFault_BootDuringEnable(boolean refresh) {
    return talonFX.getStickyFault_BootDuringEnable(refresh);
  }

  /**
   * An unlicensed feature is in use, device may not behave as expected.
   *
   * <ul>
   *   <li><b>Default Value:</b> False
   * </ul>
   *
   * <p>Default Rates:
   *
   * <ul>
   *   <li><b>CAN:</b> 4.0 Hz
   * </ul>
   *
   * <p>This refreshes and returns a cached StatusSignal object.
   *
   * @return Fault_UnlicensedFeatureInUse Status Signal Object
   */
  public StatusSignal<Boolean> getFault_UnlicensedFeatureInUse() {
    return talonFX.getFault_UnlicensedFeatureInUse();
  }

  /**
   * An unlicensed feature is in use, device may not behave as expected.
   *
   * <ul>
   *   <li><b>Default Value:</b> False
   * </ul>
   *
   * <p>Default Rates:
   *
   * <ul>
   *   <li><b>CAN:</b> 4.0 Hz
   * </ul>
   *
   * <p>This refreshes and returns a cached StatusSignal object.
   *
   * @param refresh Whether to refresh the StatusSignal before returning it; defaults to true
   * @return Fault_UnlicensedFeatureInUse Status Signal Object
   */
  public StatusSignal<Boolean> getFault_UnlicensedFeatureInUse(boolean refresh) {
    return talonFX.getFault_UnlicensedFeatureInUse(refresh);
  }

  /**
   * An unlicensed feature is in use, device may not behave as expected.
   *
   * <ul>
   *   <li><b>Default Value:</b> False
   * </ul>
   *
   * <p>Default Rates:
   *
   * <ul>
   *   <li><b>CAN:</b> 4.0 Hz
   * </ul>
   *
   * <p>This refreshes and returns a cached StatusSignal object.
   *
   * @return StickyFault_UnlicensedFeatureInUse Status Signal Object
   */
  public StatusSignal<Boolean> getStickyFault_UnlicensedFeatureInUse() {
    return talonFX.getStickyFault_UnlicensedFeatureInUse();
  }

  /**
   * An unlicensed feature is in use, device may not behave as expected.
   *
   * <ul>
   *   <li><b>Default Value:</b> False
   * </ul>
   *
   * <p>Default Rates:
   *
   * <ul>
   *   <li><b>CAN:</b> 4.0 Hz
   * </ul>
   *
   * <p>This refreshes and returns a cached StatusSignal object.
   *
   * @param refresh Whether to refresh the StatusSignal before returning it; defaults to true
   * @return StickyFault_UnlicensedFeatureInUse Status Signal Object
   */
  public StatusSignal<Boolean> getStickyFault_UnlicensedFeatureInUse(boolean refresh) {
    return talonFX.getStickyFault_UnlicensedFeatureInUse(refresh);
  }

  /**
   * Bridge was disabled most likely due to supply voltage dropping too low.
   *
   * <ul>
   *   <li><b>Default Value:</b> False
   * </ul>
   *
   * <p>Default Rates:
   *
   * <ul>
   *   <li><b>CAN:</b> 4.0 Hz
   * </ul>
   *
   * <p>This refreshes and returns a cached StatusSignal object.
   *
   * @return Fault_BridgeBrownout Status Signal Object
   */
  public StatusSignal<Boolean> getFault_BridgeBrownout() {
    return talonFX.getFault_BridgeBrownout();
  }

  /**
   * Bridge was disabled most likely due to supply voltage dropping too low.
   *
   * <ul>
   *   <li><b>Default Value:</b> False
   * </ul>
   *
   * <p>Default Rates:
   *
   * <ul>
   *   <li><b>CAN:</b> 4.0 Hz
   * </ul>
   *
   * <p>This refreshes and returns a cached StatusSignal object.
   *
   * @param refresh Whether to refresh the StatusSignal before returning it; defaults to true
   * @return Fault_BridgeBrownout Status Signal Object
   */
  public StatusSignal<Boolean> getFault_BridgeBrownout(boolean refresh) {
    return talonFX.getFault_BridgeBrownout(refresh);
  }

  /**
   * Bridge was disabled most likely due to supply voltage dropping too low.
   *
   * <ul>
   *   <li><b>Default Value:</b> False
   * </ul>
   *
   * <p>Default Rates:
   *
   * <ul>
   *   <li><b>CAN:</b> 4.0 Hz
   * </ul>
   *
   * <p>This refreshes and returns a cached StatusSignal object.
   *
   * @return StickyFault_BridgeBrownout Status Signal Object
   */
  public StatusSignal<Boolean> getStickyFault_BridgeBrownout() {
    return talonFX.getStickyFault_BridgeBrownout();
  }

  /**
   * Bridge was disabled most likely due to supply voltage dropping too low.
   *
   * <ul>
   *   <li><b>Default Value:</b> False
   * </ul>
   *
   * <p>Default Rates:
   *
   * <ul>
   *   <li><b>CAN:</b> 4.0 Hz
   * </ul>
   *
   * <p>This refreshes and returns a cached StatusSignal object.
   *
   * @param refresh Whether to refresh the StatusSignal before returning it; defaults to true
   * @return StickyFault_BridgeBrownout Status Signal Object
   */
  public StatusSignal<Boolean> getStickyFault_BridgeBrownout(boolean refresh) {
    return talonFX.getStickyFault_BridgeBrownout(refresh);
  }

  /**
   * The remote sensor has reset.
   *
   * <ul>
   *   <li><b>Default Value:</b> False
   * </ul>
   *
   * <p>Default Rates:
   *
   * <ul>
   *   <li><b>CAN:</b> 4.0 Hz
   * </ul>
   *
   * <p>This refreshes and returns a cached StatusSignal object.
   *
   * @return Fault_RemoteSensorReset Status Signal Object
   */
  public StatusSignal<Boolean> getFault_RemoteSensorReset() {
    return talonFX.getFault_RemoteSensorReset();
  }

  /**
   * The remote sensor has reset.
   *
   * <ul>
   *   <li><b>Default Value:</b> False
   * </ul>
   *
   * <p>Default Rates:
   *
   * <ul>
   *   <li><b>CAN:</b> 4.0 Hz
   * </ul>
   *
   * <p>This refreshes and returns a cached StatusSignal object.
   *
   * @param refresh Whether to refresh the StatusSignal before returning it; defaults to true
   * @return Fault_RemoteSensorReset Status Signal Object
   */
  public StatusSignal<Boolean> getFault_RemoteSensorReset(boolean refresh) {
    return talonFX.getFault_RemoteSensorReset(refresh);
  }

  /**
   * The remote sensor has reset.
   *
   * <ul>
   *   <li><b>Default Value:</b> False
   * </ul>
   *
   * <p>Default Rates:
   *
   * <ul>
   *   <li><b>CAN:</b> 4.0 Hz
   * </ul>
   *
   * <p>This refreshes and returns a cached StatusSignal object.
   *
   * @return StickyFault_RemoteSensorReset Status Signal Object
   */
  public StatusSignal<Boolean> getStickyFault_RemoteSensorReset() {
    return talonFX.getStickyFault_RemoteSensorReset();
  }

  /**
   * The remote sensor has reset.
   *
   * <ul>
   *   <li><b>Default Value:</b> False
   * </ul>
   *
   * <p>Default Rates:
   *
   * <ul>
   *   <li><b>CAN:</b> 4.0 Hz
   * </ul>
   *
   * <p>This refreshes and returns a cached StatusSignal object.
   *
   * @param refresh Whether to refresh the StatusSignal before returning it; defaults to true
   * @return StickyFault_RemoteSensorReset Status Signal Object
   */
  public StatusSignal<Boolean> getStickyFault_RemoteSensorReset(boolean refresh) {
    return talonFX.getStickyFault_RemoteSensorReset(refresh);
  }

  /**
   * The remote Talon used for differential control is not present on CAN Bus.
   *
   * <ul>
   *   <li><b>Default Value:</b> False
   * </ul>
   *
   * <p>Default Rates:
   *
   * <ul>
   *   <li><b>CAN:</b> 4.0 Hz
   * </ul>
   *
   * <p>This refreshes and returns a cached StatusSignal object.
   *
   * @return Fault_MissingDifferentialFX Status Signal Object
   */
  public StatusSignal<Boolean> getFault_MissingDifferentialFX() {
    return talonFX.getFault_MissingDifferentialFX();
  }

  /**
   * The remote Talon used for differential control is not present on CAN Bus.
   *
   * <ul>
   *   <li><b>Default Value:</b> False
   * </ul>
   *
   * <p>Default Rates:
   *
   * <ul>
   *   <li><b>CAN:</b> 4.0 Hz
   * </ul>
   *
   * <p>This refreshes and returns a cached StatusSignal object.
   *
   * @param refresh Whether to refresh the StatusSignal before returning it; defaults to true
   * @return Fault_MissingDifferentialFX Status Signal Object
   */
  public StatusSignal<Boolean> getFault_MissingDifferentialFX(boolean refresh) {
    return talonFX.getFault_MissingDifferentialFX(refresh);
  }

  /**
   * The remote Talon used for differential control is not present on CAN Bus.
   *
   * <ul>
   *   <li><b>Default Value:</b> False
   * </ul>
   *
   * <p>Default Rates:
   *
   * <ul>
   *   <li><b>CAN:</b> 4.0 Hz
   * </ul>
   *
   * <p>This refreshes and returns a cached StatusSignal object.
   *
   * @return StickyFault_MissingDifferentialFX Status Signal Object
   */
  public StatusSignal<Boolean> getStickyFault_MissingDifferentialFX() {
    return talonFX.getStickyFault_MissingDifferentialFX();
  }

  /**
   * The remote Talon used for differential control is not present on CAN Bus.
   *
   * <ul>
   *   <li><b>Default Value:</b> False
   * </ul>
   *
   * <p>Default Rates:
   *
   * <ul>
   *   <li><b>CAN:</b> 4.0 Hz
   * </ul>
   *
   * <p>This refreshes and returns a cached StatusSignal object.
   *
   * @param refresh Whether to refresh the StatusSignal before returning it; defaults to true
   * @return StickyFault_MissingDifferentialFX Status Signal Object
   */
  public StatusSignal<Boolean> getStickyFault_MissingDifferentialFX(boolean refresh) {
    return talonFX.getStickyFault_MissingDifferentialFX(refresh);
  }

  /**
   * The remote sensor position has overflowed. Because of the nature of remote sensors, it is
   * possible for the remote sensor position to overflow beyond what is supported by the status
   * signal frame. However, this is rare and cannot occur over the course of an FRC match under
   * normal use.
   *
   * <ul>
   *   <li><b>Default Value:</b> False
   * </ul>
   *
   * <p>Default Rates:
   *
   * <ul>
   *   <li><b>CAN:</b> 4.0 Hz
   * </ul>
   *
   * <p>This refreshes and returns a cached StatusSignal object.
   *
   * @return Fault_RemoteSensorPosOverflow Status Signal Object
   */
  public StatusSignal<Boolean> getFault_RemoteSensorPosOverflow() {
    return talonFX.getFault_RemoteSensorPosOverflow();
  }

  /**
   * The remote sensor position has overflowed. Because of the nature of remote sensors, it is
   * possible for the remote sensor position to overflow beyond what is supported by the status
   * signal frame. However, this is rare and cannot occur over the course of an FRC match under
   * normal use.
   *
   * <ul>
   *   <li><b>Default Value:</b> False
   * </ul>
   *
   * <p>Default Rates:
   *
   * <ul>
   *   <li><b>CAN:</b> 4.0 Hz
   * </ul>
   *
   * <p>This refreshes and returns a cached StatusSignal object.
   *
   * @param refresh Whether to refresh the StatusSignal before returning it; defaults to true
   * @return Fault_RemoteSensorPosOverflow Status Signal Object
   */
  public StatusSignal<Boolean> getFault_RemoteSensorPosOverflow(boolean refresh) {
    return talonFX.getFault_RemoteSensorPosOverflow(refresh);
  }

  /**
   * The remote sensor position has overflowed. Because of the nature of remote sensors, it is
   * possible for the remote sensor position to overflow beyond what is supported by the status
   * signal frame. However, this is rare and cannot occur over the course of an FRC match under
   * normal use.
   *
   * <ul>
   *   <li><b>Default Value:</b> False
   * </ul>
   *
   * <p>Default Rates:
   *
   * <ul>
   *   <li><b>CAN:</b> 4.0 Hz
   * </ul>
   *
   * <p>This refreshes and returns a cached StatusSignal object.
   *
   * @return StickyFault_RemoteSensorPosOverflow Status Signal Object
   */
  public StatusSignal<Boolean> getStickyFault_RemoteSensorPosOverflow() {
    return talonFX.getStickyFault_RemoteSensorPosOverflow();
  }

  /**
   * The remote sensor position has overflowed. Because of the nature of remote sensors, it is
   * possible for the remote sensor position to overflow beyond what is supported by the status
   * signal frame. However, this is rare and cannot occur over the course of an FRC match under
   * normal use.
   *
   * <ul>
   *   <li><b>Default Value:</b> False
   * </ul>
   *
   * <p>Default Rates:
   *
   * <ul>
   *   <li><b>CAN:</b> 4.0 Hz
   * </ul>
   *
   * <p>This refreshes and returns a cached StatusSignal object.
   *
   * @param refresh Whether to refresh the StatusSignal before returning it; defaults to true
   * @return StickyFault_RemoteSensorPosOverflow Status Signal Object
   */
  public StatusSignal<Boolean> getStickyFault_RemoteSensorPosOverflow(boolean refresh) {
    return talonFX.getStickyFault_RemoteSensorPosOverflow(refresh);
  }

  /**
   * Supply Voltage has exceeded the maximum voltage rating of device.
   *
   * <ul>
   *   <li><b>Default Value:</b> False
   * </ul>
   *
   * <p>Default Rates:
   *
   * <ul>
   *   <li><b>CAN:</b> 4.0 Hz
   * </ul>
   *
   * <p>This refreshes and returns a cached StatusSignal object.
   *
   * @return Fault_OverSupplyV Status Signal Object
   */
  public StatusSignal<Boolean> getFault_OverSupplyV() {
    return talonFX.getFault_OverSupplyV();
  }

  /**
   * Supply Voltage has exceeded the maximum voltage rating of device.
   *
   * <ul>
   *   <li><b>Default Value:</b> False
   * </ul>
   *
   * <p>Default Rates:
   *
   * <ul>
   *   <li><b>CAN:</b> 4.0 Hz
   * </ul>
   *
   * <p>This refreshes and returns a cached StatusSignal object.
   *
   * @param refresh Whether to refresh the StatusSignal before returning it; defaults to true
   * @return Fault_OverSupplyV Status Signal Object
   */
  public StatusSignal<Boolean> getFault_OverSupplyV(boolean refresh) {
    return talonFX.getFault_OverSupplyV(refresh);
  }

  /**
   * Supply Voltage has exceeded the maximum voltage rating of device.
   *
   * <ul>
   *   <li><b>Default Value:</b> False
   * </ul>
   *
   * <p>Default Rates:
   *
   * <ul>
   *   <li><b>CAN:</b> 4.0 Hz
   * </ul>
   *
   * <p>This refreshes and returns a cached StatusSignal object.
   *
   * @return StickyFault_OverSupplyV Status Signal Object
   */
  public StatusSignal<Boolean> getStickyFault_OverSupplyV() {
    return talonFX.getStickyFault_OverSupplyV();
  }

  /**
   * Supply Voltage has exceeded the maximum voltage rating of device.
   *
   * <ul>
   *   <li><b>Default Value:</b> False
   * </ul>
   *
   * <p>Default Rates:
   *
   * <ul>
   *   <li><b>CAN:</b> 4.0 Hz
   * </ul>
   *
   * <p>This refreshes and returns a cached StatusSignal object.
   *
   * @param refresh Whether to refresh the StatusSignal before returning it; defaults to true
   * @return StickyFault_OverSupplyV Status Signal Object
   */
  public StatusSignal<Boolean> getStickyFault_OverSupplyV(boolean refresh) {
    return talonFX.getStickyFault_OverSupplyV(refresh);
  }

  /**
   * Supply Voltage is unstable. Ensure you are using a battery and current limited power supply.
   *
   * <ul>
   *   <li><b>Default Value:</b> False
   * </ul>
   *
   * <p>Default Rates:
   *
   * <ul>
   *   <li><b>CAN:</b> 4.0 Hz
   * </ul>
   *
   * <p>This refreshes and returns a cached StatusSignal object.
   *
   * @return Fault_UnstableSupplyV Status Signal Object
   */
  public StatusSignal<Boolean> getFault_UnstableSupplyV() {
    return talonFX.getFault_UnstableSupplyV();
  }

  /**
   * Supply Voltage is unstable. Ensure you are using a battery and current limited power supply.
   *
   * <ul>
   *   <li><b>Default Value:</b> False
   * </ul>
   *
   * <p>Default Rates:
   *
   * <ul>
   *   <li><b>CAN:</b> 4.0 Hz
   * </ul>
   *
   * <p>This refreshes and returns a cached StatusSignal object.
   *
   * @param refresh Whether to refresh the StatusSignal before returning it; defaults to true
   * @return Fault_UnstableSupplyV Status Signal Object
   */
  public StatusSignal<Boolean> getFault_UnstableSupplyV(boolean refresh) {
    return talonFX.getFault_UnstableSupplyV(refresh);
  }

  /**
   * Supply Voltage is unstable. Ensure you are using a battery and current limited power supply.
   *
   * <ul>
   *   <li><b>Default Value:</b> False
   * </ul>
   *
   * <p>Default Rates:
   *
   * <ul>
   *   <li><b>CAN:</b> 4.0 Hz
   * </ul>
   *
   * <p>This refreshes and returns a cached StatusSignal object.
   *
   * @return StickyFault_UnstableSupplyV Status Signal Object
   */
  public StatusSignal<Boolean> getStickyFault_UnstableSupplyV() {
    return talonFX.getStickyFault_UnstableSupplyV();
  }

  /**
   * Supply Voltage is unstable. Ensure you are using a battery and current limited power supply.
   *
   * <ul>
   *   <li><b>Default Value:</b> False
   * </ul>
   *
   * <p>Default Rates:
   *
   * <ul>
   *   <li><b>CAN:</b> 4.0 Hz
   * </ul>
   *
   * <p>This refreshes and returns a cached StatusSignal object.
   *
   * @param refresh Whether to refresh the StatusSignal before returning it; defaults to true
   * @return StickyFault_UnstableSupplyV Status Signal Object
   */
  public StatusSignal<Boolean> getStickyFault_UnstableSupplyV(boolean refresh) {
    return talonFX.getStickyFault_UnstableSupplyV(refresh);
  }

  /**
   * Reverse limit switch has been asserted. Output is set to neutral.
   *
   * <ul>
   *   <li><b>Default Value:</b> False
   * </ul>
   *
   * <p>Default Rates:
   *
   * <ul>
   *   <li><b>CAN:</b> 4.0 Hz
   * </ul>
   *
   * <p>This refreshes and returns a cached StatusSignal object.
   *
   * @return Fault_ReverseHardLimit Status Signal Object
   */
  public StatusSignal<Boolean> getFault_ReverseHardLimit() {
    return talonFX.getFault_ReverseHardLimit();
  }

  /**
   * Reverse limit switch has been asserted. Output is set to neutral.
   *
   * <ul>
   *   <li><b>Default Value:</b> False
   * </ul>
   *
   * <p>Default Rates:
   *
   * <ul>
   *   <li><b>CAN:</b> 4.0 Hz
   * </ul>
   *
   * <p>This refreshes and returns a cached StatusSignal object.
   *
   * @param refresh Whether to refresh the StatusSignal before returning it; defaults to true
   * @return Fault_ReverseHardLimit Status Signal Object
   */
  public StatusSignal<Boolean> getFault_ReverseHardLimit(boolean refresh) {
    return talonFX.getFault_ReverseHardLimit(refresh);
  }

  /**
   * Reverse limit switch has been asserted. Output is set to neutral.
   *
   * <ul>
   *   <li><b>Default Value:</b> False
   * </ul>
   *
   * <p>Default Rates:
   *
   * <ul>
   *   <li><b>CAN:</b> 4.0 Hz
   * </ul>
   *
   * <p>This refreshes and returns a cached StatusSignal object.
   *
   * @return StickyFault_ReverseHardLimit Status Signal Object
   */
  public StatusSignal<Boolean> getStickyFault_ReverseHardLimit() {
    return talonFX.getStickyFault_ReverseHardLimit();
  }

  /**
   * Reverse limit switch has been asserted. Output is set to neutral.
   *
   * <ul>
   *   <li><b>Default Value:</b> False
   * </ul>
   *
   * <p>Default Rates:
   *
   * <ul>
   *   <li><b>CAN:</b> 4.0 Hz
   * </ul>
   *
   * <p>This refreshes and returns a cached StatusSignal object.
   *
   * @param refresh Whether to refresh the StatusSignal before returning it; defaults to true
   * @return StickyFault_ReverseHardLimit Status Signal Object
   */
  public StatusSignal<Boolean> getStickyFault_ReverseHardLimit(boolean refresh) {
    return talonFX.getStickyFault_ReverseHardLimit(refresh);
  }

  /**
   * Forward limit switch has been asserted. Output is set to neutral.
   *
   * <ul>
   *   <li><b>Default Value:</b> False
   * </ul>
   *
   * <p>Default Rates:
   *
   * <ul>
   *   <li><b>CAN:</b> 4.0 Hz
   * </ul>
   *
   * <p>This refreshes and returns a cached StatusSignal object.
   *
   * @return Fault_ForwardHardLimit Status Signal Object
   */
  public StatusSignal<Boolean> getFault_ForwardHardLimit() {
    return talonFX.getFault_ForwardHardLimit();
  }

  /**
   * Forward limit switch has been asserted. Output is set to neutral.
   *
   * <ul>
   *   <li><b>Default Value:</b> False
   * </ul>
   *
   * <p>Default Rates:
   *
   * <ul>
   *   <li><b>CAN:</b> 4.0 Hz
   * </ul>
   *
   * <p>This refreshes and returns a cached StatusSignal object.
   *
   * @param refresh Whether to refresh the StatusSignal before returning it; defaults to true
   * @return Fault_ForwardHardLimit Status Signal Object
   */
  public StatusSignal<Boolean> getFault_ForwardHardLimit(boolean refresh) {
    return talonFX.getFault_ForwardHardLimit(refresh);
  }

  /**
   * Forward limit switch has been asserted. Output is set to neutral.
   *
   * <ul>
   *   <li><b>Default Value:</b> False
   * </ul>
   *
   * <p>Default Rates:
   *
   * <ul>
   *   <li><b>CAN:</b> 4.0 Hz
   * </ul>
   *
   * <p>This refreshes and returns a cached StatusSignal object.
   *
   * @return StickyFault_ForwardHardLimit Status Signal Object
   */
  public StatusSignal<Boolean> getStickyFault_ForwardHardLimit() {
    return talonFX.getStickyFault_ForwardHardLimit();
  }

  /**
   * Forward limit switch has been asserted. Output is set to neutral.
   *
   * <ul>
   *   <li><b>Default Value:</b> False
   * </ul>
   *
   * <p>Default Rates:
   *
   * <ul>
   *   <li><b>CAN:</b> 4.0 Hz
   * </ul>
   *
   * <p>This refreshes and returns a cached StatusSignal object.
   *
   * @param refresh Whether to refresh the StatusSignal before returning it; defaults to true
   * @return StickyFault_ForwardHardLimit Status Signal Object
   */
  public StatusSignal<Boolean> getStickyFault_ForwardHardLimit(boolean refresh) {
    return talonFX.getStickyFault_ForwardHardLimit(refresh);
  }

  /**
   * Reverse soft limit has been asserted. Output is set to neutral.
   *
   * <ul>
   *   <li><b>Default Value:</b> False
   * </ul>
   *
   * <p>Default Rates:
   *
   * <ul>
   *   <li><b>CAN:</b> 4.0 Hz
   * </ul>
   *
   * <p>This refreshes and returns a cached StatusSignal object.
   *
   * @return Fault_ReverseSoftLimit Status Signal Object
   */
  public StatusSignal<Boolean> getFault_ReverseSoftLimit() {
    return talonFX.getFault_ReverseSoftLimit();
  }

  /**
   * Reverse soft limit has been asserted. Output is set to neutral.
   *
   * <ul>
   *   <li><b>Default Value:</b> False
   * </ul>
   *
   * <p>Default Rates:
   *
   * <ul>
   *   <li><b>CAN:</b> 4.0 Hz
   * </ul>
   *
   * <p>This refreshes and returns a cached StatusSignal object.
   *
   * @param refresh Whether to refresh the StatusSignal before returning it; defaults to true
   * @return Fault_ReverseSoftLimit Status Signal Object
   */
  public StatusSignal<Boolean> getFault_ReverseSoftLimit(boolean refresh) {
    return talonFX.getFault_ReverseSoftLimit(refresh);
  }

  /**
   * Reverse soft limit has been asserted. Output is set to neutral.
   *
   * <ul>
   *   <li><b>Default Value:</b> False
   * </ul>
   *
   * <p>Default Rates:
   *
   * <ul>
   *   <li><b>CAN:</b> 4.0 Hz
   * </ul>
   *
   * <p>This refreshes and returns a cached StatusSignal object.
   *
   * @return StickyFault_ReverseSoftLimit Status Signal Object
   */
  public StatusSignal<Boolean> getStickyFault_ReverseSoftLimit() {
    return talonFX.getStickyFault_ReverseSoftLimit();
  }

  /**
   * Reverse soft limit has been asserted. Output is set to neutral.
   *
   * <ul>
   *   <li><b>Default Value:</b> False
   * </ul>
   *
   * <p>Default Rates:
   *
   * <ul>
   *   <li><b>CAN:</b> 4.0 Hz
   * </ul>
   *
   * <p>This refreshes and returns a cached StatusSignal object.
   *
   * @param refresh Whether to refresh the StatusSignal before returning it; defaults to true
   * @return StickyFault_ReverseSoftLimit Status Signal Object
   */
  public StatusSignal<Boolean> getStickyFault_ReverseSoftLimit(boolean refresh) {
    return talonFX.getStickyFault_ReverseSoftLimit(refresh);
  }

  /**
   * Forward soft limit has been asserted. Output is set to neutral.
   *
   * <ul>
   *   <li><b>Default Value:</b> False
   * </ul>
   *
   * <p>Default Rates:
   *
   * <ul>
   *   <li><b>CAN:</b> 4.0 Hz
   * </ul>
   *
   * <p>This refreshes and returns a cached StatusSignal object.
   *
   * @return Fault_ForwardSoftLimit Status Signal Object
   */
  public StatusSignal<Boolean> getFault_ForwardSoftLimit() {
    return talonFX.getFault_ForwardSoftLimit();
  }

  /**
   * Forward soft limit has been asserted. Output is set to neutral.
   *
   * <ul>
   *   <li><b>Default Value:</b> False
   * </ul>
   *
   * <p>Default Rates:
   *
   * <ul>
   *   <li><b>CAN:</b> 4.0 Hz
   * </ul>
   *
   * <p>This refreshes and returns a cached StatusSignal object.
   *
   * @param refresh Whether to refresh the StatusSignal before returning it; defaults to true
   * @return Fault_ForwardSoftLimit Status Signal Object
   */
  public StatusSignal<Boolean> getFault_ForwardSoftLimit(boolean refresh) {
    return talonFX.getFault_ForwardSoftLimit(refresh);
  }

  /**
   * Forward soft limit has been asserted. Output is set to neutral.
   *
   * <ul>
   *   <li><b>Default Value:</b> False
   * </ul>
   *
   * <p>Default Rates:
   *
   * <ul>
   *   <li><b>CAN:</b> 4.0 Hz
   * </ul>
   *
   * <p>This refreshes and returns a cached StatusSignal object.
   *
   * @return StickyFault_ForwardSoftLimit Status Signal Object
   */
  public StatusSignal<Boolean> getStickyFault_ForwardSoftLimit() {
    return talonFX.getStickyFault_ForwardSoftLimit();
  }

  /**
   * Forward soft limit has been asserted. Output is set to neutral.
   *
   * <ul>
   *   <li><b>Default Value:</b> False
   * </ul>
   *
   * <p>Default Rates:
   *
   * <ul>
   *   <li><b>CAN:</b> 4.0 Hz
   * </ul>
   *
   * <p>This refreshes and returns a cached StatusSignal object.
   *
   * @param refresh Whether to refresh the StatusSignal before returning it; defaults to true
   * @return StickyFault_ForwardSoftLimit Status Signal Object
   */
  public StatusSignal<Boolean> getStickyFault_ForwardSoftLimit(boolean refresh) {
    return talonFX.getStickyFault_ForwardSoftLimit(refresh);
  }

  /**
   * The remote soft limit device is not present on CAN Bus.
   *
   * <ul>
   *   <li><b>Default Value:</b> False
   * </ul>
   *
   * <p>Default Rates:
   *
   * <ul>
   *   <li><b>CAN:</b> 4.0 Hz
   * </ul>
   *
   * <p>This refreshes and returns a cached StatusSignal object.
   *
   * @return Fault_MissingSoftLimitRemote Status Signal Object
   */
  public StatusSignal<Boolean> getFault_MissingSoftLimitRemote() {
    return talonFX.getFault_MissingSoftLimitRemote();
  }

  /**
   * The remote soft limit device is not present on CAN Bus.
   *
   * <ul>
   *   <li><b>Default Value:</b> False
   * </ul>
   *
   * <p>Default Rates:
   *
   * <ul>
   *   <li><b>CAN:</b> 4.0 Hz
   * </ul>
   *
   * <p>This refreshes and returns a cached StatusSignal object.
   *
   * @param refresh Whether to refresh the StatusSignal before returning it; defaults to true
   * @return Fault_MissingSoftLimitRemote Status Signal Object
   */
  public StatusSignal<Boolean> getFault_MissingSoftLimitRemote(boolean refresh) {
    return talonFX.getFault_MissingSoftLimitRemote(refresh);
  }

  /**
   * The remote soft limit device is not present on CAN Bus.
   *
   * <ul>
   *   <li><b>Default Value:</b> False
   * </ul>
   *
   * <p>Default Rates:
   *
   * <ul>
   *   <li><b>CAN:</b> 4.0 Hz
   * </ul>
   *
   * <p>This refreshes and returns a cached StatusSignal object.
   *
   * @return StickyFault_MissingSoftLimitRemote Status Signal Object
   */
  public StatusSignal<Boolean> getStickyFault_MissingSoftLimitRemote() {
    return talonFX.getStickyFault_MissingSoftLimitRemote();
  }

  /**
   * The remote soft limit device is not present on CAN Bus.
   *
   * <ul>
   *   <li><b>Default Value:</b> False
   * </ul>
   *
   * <p>Default Rates:
   *
   * <ul>
   *   <li><b>CAN:</b> 4.0 Hz
   * </ul>
   *
   * <p>This refreshes and returns a cached StatusSignal object.
   *
   * @param refresh Whether to refresh the StatusSignal before returning it; defaults to true
   * @return StickyFault_MissingSoftLimitRemote Status Signal Object
   */
  public StatusSignal<Boolean> getStickyFault_MissingSoftLimitRemote(boolean refresh) {
    return talonFX.getStickyFault_MissingSoftLimitRemote(refresh);
  }

  /**
   * The remote limit switch device is not present on CAN Bus.
   *
   * <ul>
   *   <li><b>Default Value:</b> False
   * </ul>
   *
   * <p>Default Rates:
   *
   * <ul>
   *   <li><b>CAN:</b> 4.0 Hz
   * </ul>
   *
   * <p>This refreshes and returns a cached StatusSignal object.
   *
   * @return Fault_MissingHardLimitRemote Status Signal Object
   */
  public StatusSignal<Boolean> getFault_MissingHardLimitRemote() {
    return talonFX.getFault_MissingHardLimitRemote();
  }

  /**
   * The remote limit switch device is not present on CAN Bus.
   *
   * <ul>
   *   <li><b>Default Value:</b> False
   * </ul>
   *
   * <p>Default Rates:
   *
   * <ul>
   *   <li><b>CAN:</b> 4.0 Hz
   * </ul>
   *
   * <p>This refreshes and returns a cached StatusSignal object.
   *
   * @param refresh Whether to refresh the StatusSignal before returning it; defaults to true
   * @return Fault_MissingHardLimitRemote Status Signal Object
   */
  public StatusSignal<Boolean> getFault_MissingHardLimitRemote(boolean refresh) {
    return talonFX.getFault_MissingHardLimitRemote(refresh);
  }

  /**
   * The remote limit switch device is not present on CAN Bus.
   *
   * <ul>
   *   <li><b>Default Value:</b> False
   * </ul>
   *
   * <p>Default Rates:
   *
   * <ul>
   *   <li><b>CAN:</b> 4.0 Hz
   * </ul>
   *
   * <p>This refreshes and returns a cached StatusSignal object.
   *
   * @return StickyFault_MissingHardLimitRemote Status Signal Object
   */
  public StatusSignal<Boolean> getStickyFault_MissingHardLimitRemote() {
    return talonFX.getStickyFault_MissingHardLimitRemote();
  }

  /**
   * The remote limit switch device is not present on CAN Bus.
   *
   * <ul>
   *   <li><b>Default Value:</b> False
   * </ul>
   *
   * <p>Default Rates:
   *
   * <ul>
   *   <li><b>CAN:</b> 4.0 Hz
   * </ul>
   *
   * <p>This refreshes and returns a cached StatusSignal object.
   *
   * @param refresh Whether to refresh the StatusSignal before returning it; defaults to true
   * @return StickyFault_MissingHardLimitRemote Status Signal Object
   */
  public StatusSignal<Boolean> getStickyFault_MissingHardLimitRemote(boolean refresh) {
    return talonFX.getStickyFault_MissingHardLimitRemote(refresh);
  }

  /**
   * The remote sensor's data is no longer trusted. This can happen if the remote sensor disappears
   * from the CAN bus or if the remote sensor indicates its data is no longer valid, such as when a
   * CANcoder's magnet strength falls into the "red" range.
   *
   * <ul>
   *   <li><b>Default Value:</b> False
   * </ul>
   *
   * <p>Default Rates:
   *
   * <ul>
   *   <li><b>CAN:</b> 4.0 Hz
   * </ul>
   *
   * <p>This refreshes and returns a cached StatusSignal object.
   *
   * @return Fault_RemoteSensorDataInvalid Status Signal Object
   */
  public StatusSignal<Boolean> getFault_RemoteSensorDataInvalid() {
    return talonFX.getFault_RemoteSensorDataInvalid();
  }

  /**
   * The remote sensor's data is no longer trusted. This can happen if the remote sensor disappears
   * from the CAN bus or if the remote sensor indicates its data is no longer valid, such as when a
   * CANcoder's magnet strength falls into the "red" range.
   *
   * <ul>
   *   <li><b>Default Value:</b> False
   * </ul>
   *
   * <p>Default Rates:
   *
   * <ul>
   *   <li><b>CAN:</b> 4.0 Hz
   * </ul>
   *
   * <p>This refreshes and returns a cached StatusSignal object.
   *
   * @param refresh Whether to refresh the StatusSignal before returning it; defaults to true
   * @return Fault_RemoteSensorDataInvalid Status Signal Object
   */
  public StatusSignal<Boolean> getFault_RemoteSensorDataInvalid(boolean refresh) {
    return talonFX.getFault_RemoteSensorDataInvalid(refresh);
  }

  /**
   * The remote sensor's data is no longer trusted. This can happen if the remote sensor disappears
   * from the CAN bus or if the remote sensor indicates its data is no longer valid, such as when a
   * CANcoder's magnet strength falls into the "red" range.
   *
   * <ul>
   *   <li><b>Default Value:</b> False
   * </ul>
   *
   * <p>Default Rates:
   *
   * <ul>
   *   <li><b>CAN:</b> 4.0 Hz
   * </ul>
   *
   * <p>This refreshes and returns a cached StatusSignal object.
   *
   * @return StickyFault_RemoteSensorDataInvalid Status Signal Object
   */
  public StatusSignal<Boolean> getStickyFault_RemoteSensorDataInvalid() {
    return talonFX.getStickyFault_RemoteSensorDataInvalid();
  }

  /**
   * The remote sensor's data is no longer trusted. This can happen if the remote sensor disappears
   * from the CAN bus or if the remote sensor indicates its data is no longer valid, such as when a
   * CANcoder's magnet strength falls into the "red" range.
   *
   * <ul>
   *   <li><b>Default Value:</b> False
   * </ul>
   *
   * <p>Default Rates:
   *
   * <ul>
   *   <li><b>CAN:</b> 4.0 Hz
   * </ul>
   *
   * <p>This refreshes and returns a cached StatusSignal object.
   *
   * @param refresh Whether to refresh the StatusSignal before returning it; defaults to true
   * @return StickyFault_RemoteSensorDataInvalid Status Signal Object
   */
  public StatusSignal<Boolean> getStickyFault_RemoteSensorDataInvalid(boolean refresh) {
    return talonFX.getStickyFault_RemoteSensorDataInvalid(refresh);
  }

  /**
   * The remote sensor used for fusion has fallen out of sync to the local sensor. A
   * re-synchronization has occurred, which may cause a discontinuity. This typically happens if
   * there is significant slop in the mechanism, or if the RotorToSensorRatio configuration
   * parameter is incorrect.
   *
   * <ul>
   *   <li><b>Default Value:</b> False
   * </ul>
   *
   * <p>Default Rates:
   *
   * <ul>
   *   <li><b>CAN:</b> 4.0 Hz
   * </ul>
   *
   * <p>This refreshes and returns a cached StatusSignal object.
   *
   * @return Fault_FusedSensorOutOfSync Status Signal Object
   */
  public StatusSignal<Boolean> getFault_FusedSensorOutOfSync() {
    return talonFX.getFault_FusedSensorOutOfSync();
  }

  /**
   * The remote sensor used for fusion has fallen out of sync to the local sensor. A
   * re-synchronization has occurred, which may cause a discontinuity. This typically happens if
   * there is significant slop in the mechanism, or if the RotorToSensorRatio configuration
   * parameter is incorrect.
   *
   * <ul>
   *   <li><b>Default Value:</b> False
   * </ul>
   *
   * <p>Default Rates:
   *
   * <ul>
   *   <li><b>CAN:</b> 4.0 Hz
   * </ul>
   *
   * <p>This refreshes and returns a cached StatusSignal object.
   *
   * @param refresh Whether to refresh the StatusSignal before returning it; defaults to true
   * @return Fault_FusedSensorOutOfSync Status Signal Object
   */
  public StatusSignal<Boolean> getFault_FusedSensorOutOfSync(boolean refresh) {
    return talonFX.getFault_FusedSensorOutOfSync(refresh);
  }

  /**
   * The remote sensor used for fusion has fallen out of sync to the local sensor. A
   * re-synchronization has occurred, which may cause a discontinuity. This typically happens if
   * there is significant slop in the mechanism, or if the RotorToSensorRatio configuration
   * parameter is incorrect.
   *
   * <ul>
   *   <li><b>Default Value:</b> False
   * </ul>
   *
   * <p>Default Rates:
   *
   * <ul>
   *   <li><b>CAN:</b> 4.0 Hz
   * </ul>
   *
   * <p>This refreshes and returns a cached StatusSignal object.
   *
   * @return StickyFault_FusedSensorOutOfSync Status Signal Object
   */
  public StatusSignal<Boolean> getStickyFault_FusedSensorOutOfSync() {
    return talonFX.getStickyFault_FusedSensorOutOfSync();
  }

  /**
   * The remote sensor used for fusion has fallen out of sync to the local sensor. A
   * re-synchronization has occurred, which may cause a discontinuity. This typically happens if
   * there is significant slop in the mechanism, or if the RotorToSensorRatio configuration
   * parameter is incorrect.
   *
   * <ul>
   *   <li><b>Default Value:</b> False
   * </ul>
   *
   * <p>Default Rates:
   *
   * <ul>
   *   <li><b>CAN:</b> 4.0 Hz
   * </ul>
   *
   * <p>This refreshes and returns a cached StatusSignal object.
   *
   * @param refresh Whether to refresh the StatusSignal before returning it; defaults to true
   * @return StickyFault_FusedSensorOutOfSync Status Signal Object
   */
  public StatusSignal<Boolean> getStickyFault_FusedSensorOutOfSync(boolean refresh) {
    return talonFX.getStickyFault_FusedSensorOutOfSync(refresh);
  }

  /**
   * Stator current limit occured.
   *
   * <ul>
   *   <li><b>Default Value:</b> False
   * </ul>
   *
   * <p>Default Rates:
   *
   * <ul>
   *   <li><b>CAN:</b> 4.0 Hz
   * </ul>
   *
   * <p>This refreshes and returns a cached StatusSignal object.
   *
   * @return Fault_StatorCurrLimit Status Signal Object
   */
  public StatusSignal<Boolean> getFault_StatorCurrLimit() {
    return talonFX.getFault_StatorCurrLimit();
  }

  /**
   * Stator current limit occured.
   *
   * <ul>
   *   <li><b>Default Value:</b> False
   * </ul>
   *
   * <p>Default Rates:
   *
   * <ul>
   *   <li><b>CAN:</b> 4.0 Hz
   * </ul>
   *
   * <p>This refreshes and returns a cached StatusSignal object.
   *
   * @param refresh Whether to refresh the StatusSignal before returning it; defaults to true
   * @return Fault_StatorCurrLimit Status Signal Object
   */
  public StatusSignal<Boolean> getFault_StatorCurrLimit(boolean refresh) {
    return talonFX.getFault_StatorCurrLimit(refresh);
  }

  /**
   * Stator current limit occured.
   *
   * <ul>
   *   <li><b>Default Value:</b> False
   * </ul>
   *
   * <p>Default Rates:
   *
   * <ul>
   *   <li><b>CAN:</b> 4.0 Hz
   * </ul>
   *
   * <p>This refreshes and returns a cached StatusSignal object.
   *
   * @return StickyFault_StatorCurrLimit Status Signal Object
   */
  public StatusSignal<Boolean> getStickyFault_StatorCurrLimit() {
    return talonFX.getStickyFault_StatorCurrLimit();
  }

  /**
   * Stator current limit occured.
   *
   * <ul>
   *   <li><b>Default Value:</b> False
   * </ul>
   *
   * <p>Default Rates:
   *
   * <ul>
   *   <li><b>CAN:</b> 4.0 Hz
   * </ul>
   *
   * <p>This refreshes and returns a cached StatusSignal object.
   *
   * @param refresh Whether to refresh the StatusSignal before returning it; defaults to true
   * @return StickyFault_StatorCurrLimit Status Signal Object
   */
  public StatusSignal<Boolean> getStickyFault_StatorCurrLimit(boolean refresh) {
    return talonFX.getStickyFault_StatorCurrLimit(refresh);
  }

  /**
   * Supply current limit occured.
   *
   * <ul>
   *   <li><b>Default Value:</b> False
   * </ul>
   *
   * <p>Default Rates:
   *
   * <ul>
   *   <li><b>CAN:</b> 4.0 Hz
   * </ul>
   *
   * <p>This refreshes and returns a cached StatusSignal object.
   *
   * @return Fault_SupplyCurrLimit Status Signal Object
   */
  public StatusSignal<Boolean> getFault_SupplyCurrLimit() {
    return talonFX.getFault_SupplyCurrLimit();
  }

  /**
   * Supply current limit occured.
   *
   * <ul>
   *   <li><b>Default Value:</b> False
   * </ul>
   *
   * <p>Default Rates:
   *
   * <ul>
   *   <li><b>CAN:</b> 4.0 Hz
   * </ul>
   *
   * <p>This refreshes and returns a cached StatusSignal object.
   *
   * @param refresh Whether to refresh the StatusSignal before returning it; defaults to true
   * @return Fault_SupplyCurrLimit Status Signal Object
   */
  public StatusSignal<Boolean> getFault_SupplyCurrLimit(boolean refresh) {
    return talonFX.getFault_SupplyCurrLimit(refresh);
  }

  /**
   * Supply current limit occured.
   *
   * <ul>
   *   <li><b>Default Value:</b> False
   * </ul>
   *
   * <p>Default Rates:
   *
   * <ul>
   *   <li><b>CAN:</b> 4.0 Hz
   * </ul>
   *
   * <p>This refreshes and returns a cached StatusSignal object.
   *
   * @return StickyFault_SupplyCurrLimit Status Signal Object
   */
  public StatusSignal<Boolean> getStickyFault_SupplyCurrLimit() {
    return talonFX.getStickyFault_SupplyCurrLimit();
  }

  /**
   * Supply current limit occured.
   *
   * <ul>
   *   <li><b>Default Value:</b> False
   * </ul>
   *
   * <p>Default Rates:
   *
   * <ul>
   *   <li><b>CAN:</b> 4.0 Hz
   * </ul>
   *
   * <p>This refreshes and returns a cached StatusSignal object.
   *
   * @param refresh Whether to refresh the StatusSignal before returning it; defaults to true
   * @return StickyFault_SupplyCurrLimit Status Signal Object
   */
  public StatusSignal<Boolean> getStickyFault_SupplyCurrLimit(boolean refresh) {
    return talonFX.getStickyFault_SupplyCurrLimit(refresh);
  }

  /**
   * Using Fused CANcoder feature while unlicensed. Device has fallen back to remote CANcoder.
   *
   * <ul>
   *   <li><b>Default Value:</b> False
   * </ul>
   *
   * <p>Default Rates:
   *
   * <ul>
   *   <li><b>CAN:</b> 4.0 Hz
   * </ul>
   *
   * <p>This refreshes and returns a cached StatusSignal object.
   *
   * @return Fault_UsingFusedCANcoderWhileUnlicensed Status Signal Object
   */
  public StatusSignal<Boolean> getFault_UsingFusedCANcoderWhileUnlicensed() {
    return talonFX.getFault_UsingFusedCANcoderWhileUnlicensed();
  }

  /**
   * Using Fused CANcoder feature while unlicensed. Device has fallen back to remote CANcoder.
   *
   * <ul>
   *   <li><b>Default Value:</b> False
   * </ul>
   *
   * <p>Default Rates:
   *
   * <ul>
   *   <li><b>CAN:</b> 4.0 Hz
   * </ul>
   *
   * <p>This refreshes and returns a cached StatusSignal object.
   *
   * @param refresh Whether to refresh the StatusSignal before returning it; defaults to true
   * @return Fault_UsingFusedCANcoderWhileUnlicensed Status Signal Object
   */
  public StatusSignal<Boolean> getFault_UsingFusedCANcoderWhileUnlicensed(boolean refresh) {
    return talonFX.getFault_UsingFusedCANcoderWhileUnlicensed(refresh);
  }

  /**
   * Using Fused CANcoder feature while unlicensed. Device has fallen back to remote CANcoder.
   *
   * <ul>
   *   <li><b>Default Value:</b> False
   * </ul>
   *
   * <p>Default Rates:
   *
   * <ul>
   *   <li><b>CAN:</b> 4.0 Hz
   * </ul>
   *
   * <p>This refreshes and returns a cached StatusSignal object.
   *
   * @return StickyFault_UsingFusedCANcoderWhileUnlicensed Status Signal Object
   */
  public StatusSignal<Boolean> getStickyFault_UsingFusedCANcoderWhileUnlicensed() {
    return talonFX.getStickyFault_UsingFusedCANcoderWhileUnlicensed();
  }

  /**
   * Using Fused CANcoder feature while unlicensed. Device has fallen back to remote CANcoder.
   *
   * <ul>
   *   <li><b>Default Value:</b> False
   * </ul>
   *
   * <p>Default Rates:
   *
   * <ul>
   *   <li><b>CAN:</b> 4.0 Hz
   * </ul>
   *
   * <p>This refreshes and returns a cached StatusSignal object.
   *
   * @param refresh Whether to refresh the StatusSignal before returning it; defaults to true
   * @return StickyFault_UsingFusedCANcoderWhileUnlicensed Status Signal Object
   */
  public StatusSignal<Boolean> getStickyFault_UsingFusedCANcoderWhileUnlicensed(boolean refresh) {
    return talonFX.getStickyFault_UsingFusedCANcoderWhileUnlicensed(refresh);
  }

  /**
   * Static brake was momentarily disabled due to excessive braking current while disabled.
   *
   * <ul>
   *   <li><b>Default Value:</b> False
   * </ul>
   *
   * <p>Default Rates:
   *
   * <ul>
   *   <li><b>CAN:</b> 4.0 Hz
   * </ul>
   *
   * <p>This refreshes and returns a cached StatusSignal object.
   *
   * @return Fault_StaticBrakeDisabled Status Signal Object
   */
  public StatusSignal<Boolean> getFault_StaticBrakeDisabled() {
    return talonFX.getFault_StaticBrakeDisabled();
  }

  /**
   * Static brake was momentarily disabled due to excessive braking current while disabled.
   *
   * <ul>
   *   <li><b>Default Value:</b> False
   * </ul>
   *
   * <p>Default Rates:
   *
   * <ul>
   *   <li><b>CAN:</b> 4.0 Hz
   * </ul>
   *
   * <p>This refreshes and returns a cached StatusSignal object.
   *
   * @param refresh Whether to refresh the StatusSignal before returning it; defaults to true
   * @return Fault_StaticBrakeDisabled Status Signal Object
   */
  public StatusSignal<Boolean> getFault_StaticBrakeDisabled(boolean refresh) {
    return talonFX.getFault_StaticBrakeDisabled(refresh);
  }

  /**
   * Static brake was momentarily disabled due to excessive braking current while disabled.
   *
   * <ul>
   *   <li><b>Default Value:</b> False
   * </ul>
   *
   * <p>Default Rates:
   *
   * <ul>
   *   <li><b>CAN:</b> 4.0 Hz
   * </ul>
   *
   * <p>This refreshes and returns a cached StatusSignal object.
   *
   * @return StickyFault_StaticBrakeDisabled Status Signal Object
   */
  public StatusSignal<Boolean> getStickyFault_StaticBrakeDisabled() {
    return talonFX.getStickyFault_StaticBrakeDisabled();
  }

  /**
   * Static brake was momentarily disabled due to excessive braking current while disabled.
   *
   * <ul>
   *   <li><b>Default Value:</b> False
   * </ul>
   *
   * <p>Default Rates:
   *
   * <ul>
   *   <li><b>CAN:</b> 4.0 Hz
   * </ul>
   *
   * <p>This refreshes and returns a cached StatusSignal object.
   *
   * @param refresh Whether to refresh the StatusSignal before returning it; defaults to true
   * @return StickyFault_StaticBrakeDisabled Status Signal Object
   */
  public StatusSignal<Boolean> getStickyFault_StaticBrakeDisabled(boolean refresh) {
    return talonFX.getStickyFault_StaticBrakeDisabled(refresh);
  }

  /**
   * Closed loop proportional component
   *
   * <p>The portion of the closed loop output that is the proportional to the error. Alternatively,
   * the p-Contribution of the closed loop output.
   *
   * <p>Default Rates:
   *
   * <ul>
   *   <li><b>CAN 2.0:</b> 4.0 Hz
   *   <li><b>CAN FD:</b> 100.0 Hz (TimeSynced with Pro)
   * </ul>
   *
   * <p>This refreshes and returns a cached StatusSignal object.
   *
   * @return ClosedLoopProportionalOutput Status Signal object
   */
  public StatusSignal<Double> getClosedLoopProportionalOutput() {
    return talonFX.getClosedLoopProportionalOutput();
  }

  /**
   * Closed loop proportional component
   *
   * <p>The portion of the closed loop output that is the proportional to the error. Alternatively,
   * the p-Contribution of the closed loop output.
   *
   * <p>Default Rates:
   *
   * <ul>
   *   <li><b>CAN 2.0:</b> 4.0 Hz
   *   <li><b>CAN FD:</b> 100.0 Hz (TimeSynced with Pro)
   * </ul>
   *
   * <p>This refreshes and returns a cached StatusSignal object.
   *
   * @param refresh Whether to refresh the StatusSignal before returning it; defaults to true
   * @return ClosedLoopProportionalOutput Status Signal object
   */
  public StatusSignal<Double> getClosedLoopProportionalOutput(boolean refresh) {
    return talonFX.getClosedLoopProportionalOutput(refresh);
  }

  /**
   * Closed loop integrated component
   *
   * <p>The portion of the closed loop output that is proportional to the integrated error.
   * Alternatively, the i-Contribution of the closed loop output.
   *
   * <p>Default Rates:
   *
   * <ul>
   *   <li><b>CAN 2.0:</b> 4.0 Hz
   *   <li><b>CAN FD:</b> 100.0 Hz (TimeSynced with Pro)
   * </ul>
   *
   * <p>This refreshes and returns a cached StatusSignal object.
   *
   * @return ClosedLoopIntegratedOutput Status Signal object
   */
  public StatusSignal<Double> getClosedLoopIntegratedOutput() {
    return talonFX.getClosedLoopIntegratedOutput();
  }

  /**
   * Closed loop integrated component
   *
   * <p>The portion of the closed loop output that is proportional to the integrated error.
   * Alternatively, the i-Contribution of the closed loop output.
   *
   * <p>Default Rates:
   *
   * <ul>
   *   <li><b>CAN 2.0:</b> 4.0 Hz
   *   <li><b>CAN FD:</b> 100.0 Hz (TimeSynced with Pro)
   * </ul>
   *
   * <p>This refreshes and returns a cached StatusSignal object.
   *
   * @param refresh Whether to refresh the StatusSignal before returning it; defaults to true
   * @return ClosedLoopIntegratedOutput Status Signal object
   */
  public StatusSignal<Double> getClosedLoopIntegratedOutput(boolean refresh) {
    return talonFX.getClosedLoopIntegratedOutput(refresh);
  }

  /**
   * Feedforward passed by the user
   *
   * <p>This is the general feedforward that the user provides for the closed loop.
   *
   * <p>Default Rates:
   *
   * <ul>
   *   <li><b>CAN 2.0:</b> 4.0 Hz
   *   <li><b>CAN FD:</b> 100.0 Hz (TimeSynced with Pro)
   * </ul>
   *
   * <p>This refreshes and returns a cached StatusSignal object.
   *
   * @return ClosedLoopFeedForward Status Signal object
   */
  public StatusSignal<Double> getClosedLoopFeedForward() {
    return talonFX.getClosedLoopFeedForward();
  }

  /**
   * Feedforward passed by the user
   *
   * <p>This is the general feedforward that the user provides for the closed loop.
   *
   * <p>Default Rates:
   *
   * <ul>
   *   <li><b>CAN 2.0:</b> 4.0 Hz
   *   <li><b>CAN FD:</b> 100.0 Hz (TimeSynced with Pro)
   * </ul>
   *
   * <p>This refreshes and returns a cached StatusSignal object.
   *
   * @param refresh Whether to refresh the StatusSignal before returning it; defaults to true
   * @return ClosedLoopFeedForward Status Signal object
   */
  public StatusSignal<Double> getClosedLoopFeedForward(boolean refresh) {
    return talonFX.getClosedLoopFeedForward(refresh);
  }

  /**
   * Closed loop derivative component
   *
   * <p>The portion of the closed loop output that is the proportional to the deriviative the error.
   * Alternatively, the d-Contribution of the closed loop output.
   *
   * <p>Default Rates:
   *
   * <ul>
   *   <li><b>CAN 2.0:</b> 4.0 Hz
   *   <li><b>CAN FD:</b> 100.0 Hz (TimeSynced with Pro)
   * </ul>
   *
   * <p>This refreshes and returns a cached StatusSignal object.
   *
   * @return ClosedLoopDerivativeOutput Status Signal object
   */
  public StatusSignal<Double> getClosedLoopDerivativeOutput() {
    return talonFX.getClosedLoopDerivativeOutput();
  }

  /**
   * Closed loop derivative component
   *
   * <p>The portion of the closed loop output that is the proportional to the deriviative the error.
   * Alternatively, the d-Contribution of the closed loop output.
   *
   * <p>Default Rates:
   *
   * <ul>
   *   <li><b>CAN 2.0:</b> 4.0 Hz
   *   <li><b>CAN FD:</b> 100.0 Hz (TimeSynced with Pro)
   * </ul>
   *
   * <p>This refreshes and returns a cached StatusSignal object.
   *
   * @param refresh Whether to refresh the StatusSignal before returning it; defaults to true
   * @return ClosedLoopDerivativeOutput Status Signal object
   */
  public StatusSignal<Double> getClosedLoopDerivativeOutput(boolean refresh) {
    return talonFX.getClosedLoopDerivativeOutput(refresh);
  }

  /**
   * Closed loop total output
   *
   * <p>The total output of the closed loop output.
   *
   * <p>Default Rates:
   *
   * <ul>
   *   <li><b>CAN 2.0:</b> 4.0 Hz
   *   <li><b>CAN FD:</b> 100.0 Hz (TimeSynced with Pro)
   * </ul>
   *
   * <p>This refreshes and returns a cached StatusSignal object.
   *
   * @return ClosedLoopOutput Status Signal object
   */
  public StatusSignal<Double> getClosedLoopOutput() {
    return talonFX.getClosedLoopOutput();
  }

  /**
   * Closed loop total output
   *
   * <p>The total output of the closed loop output.
   *
   * <p>Default Rates:
   *
   * <ul>
   *   <li><b>CAN 2.0:</b> 4.0 Hz
   *   <li><b>CAN FD:</b> 100.0 Hz (TimeSynced with Pro)
   * </ul>
   *
   * <p>This refreshes and returns a cached StatusSignal object.
   *
   * @param refresh Whether to refresh the StatusSignal before returning it; defaults to true
   * @return ClosedLoopOutput Status Signal object
   */
  public StatusSignal<Double> getClosedLoopOutput(boolean refresh) {
    return talonFX.getClosedLoopOutput(refresh);
  }

  /**
   * Value that the closed loop is targeting
   *
   * <p>This is the value that the closed loop PID controller targets.
   *
   * <p>Default Rates:
   *
   * <ul>
   *   <li><b>CAN 2.0:</b> 4.0 Hz
   *   <li><b>CAN FD:</b> 100.0 Hz (TimeSynced with Pro)
   * </ul>
   *
   * <p>This refreshes and returns a cached StatusSignal object.
   *
   * @return ClosedLoopReference Status Signal object
   */
  public StatusSignal<Double> getClosedLoopReference() {
    return talonFX.getClosedLoopReference();
  }

  /**
   * Value that the closed loop is targeting
   *
   * <p>This is the value that the closed loop PID controller targets.
   *
   * <p>Default Rates:
   *
   * <ul>
   *   <li><b>CAN 2.0:</b> 4.0 Hz
   *   <li><b>CAN FD:</b> 100.0 Hz (TimeSynced with Pro)
   * </ul>
   *
   * <p>This refreshes and returns a cached StatusSignal object.
   *
   * @param refresh Whether to refresh the StatusSignal before returning it; defaults to true
   * @return ClosedLoopReference Status Signal object
   */
  public StatusSignal<Double> getClosedLoopReference(boolean refresh) {
    return talonFX.getClosedLoopReference(refresh);
  }

  /**
   * Derivative of the target that the closed loop is targeting
   *
   * <p>This is the change in the closed loop reference. This may be used in the feed-forward
   * calculation, the derivative-error, or in application of the signage for kS. Typically, this
   * represents the target velocity during Motion Magic®.
   *
   * <p>Default Rates:
   *
   * <ul>
   *   <li><b>CAN 2.0:</b> 4.0 Hz
   *   <li><b>CAN FD:</b> 100.0 Hz (TimeSynced with Pro)
   * </ul>
   *
   * <p>This refreshes and returns a cached StatusSignal object.
   *
   * @return ClosedLoopReferenceSlope Status Signal object
   */
  public StatusSignal<Double> getClosedLoopReferenceSlope() {
    return talonFX.getClosedLoopReferenceSlope();
  }

  /**
   * Derivative of the target that the closed loop is targeting
   *
   * <p>This is the change in the closed loop reference. This may be used in the feed-forward
   * calculation, the derivative-error, or in application of the signage for kS. Typically, this
   * represents the target velocity during Motion Magic®.
   *
   * <p>Default Rates:
   *
   * <ul>
   *   <li><b>CAN 2.0:</b> 4.0 Hz
   *   <li><b>CAN FD:</b> 100.0 Hz (TimeSynced with Pro)
   * </ul>
   *
   * <p>This refreshes and returns a cached StatusSignal object.
   *
   * @param refresh Whether to refresh the StatusSignal before returning it; defaults to true
   * @return ClosedLoopReferenceSlope Status Signal object
   */
  public StatusSignal<Double> getClosedLoopReferenceSlope(boolean refresh) {
    return talonFX.getClosedLoopReferenceSlope(refresh);
  }

  /**
   * The difference between target reference and current measurement
   *
   * <p>This is the value that is treated as the error in the PID loop.
   *
   * <p>Default Rates:
   *
   * <ul>
   *   <li><b>CAN 2.0:</b> 4.0 Hz
   *   <li><b>CAN FD:</b> 100.0 Hz (TimeSynced with Pro)
   * </ul>
   *
   * <p>This refreshes and returns a cached StatusSignal object.
   *
   * @return ClosedLoopError Status Signal object
   */
  public StatusSignal<Double> getClosedLoopError() {
    return talonFX.getClosedLoopError();
  }

  /**
   * The difference between target reference and current measurement
   *
   * <p>This is the value that is treated as the error in the PID loop.
   *
   * <p>Default Rates:
   *
   * <ul>
   *   <li><b>CAN 2.0:</b> 4.0 Hz
   *   <li><b>CAN FD:</b> 100.0 Hz (TimeSynced with Pro)
   * </ul>
   *
   * <p>This refreshes and returns a cached StatusSignal object.
   *
   * @param refresh Whether to refresh the StatusSignal before returning it; defaults to true
   * @return ClosedLoopError Status Signal object
   */
  public StatusSignal<Double> getClosedLoopError(boolean refresh) {
    return talonFX.getClosedLoopError(refresh);
  }

  /**
   * The calculated motor output for differential followers.
   *
   * <p>This is a torque request when using the TorqueCurrentFOC control output type, and a duty
   * cycle in all other control types.
   *
   * <p>Default Rates:
   *
   * <ul>
   *   <li><b>CAN 2.0:</b> 100.0 Hz
   *   <li><b>CAN FD:</b> 100.0 Hz (TimeSynced with Pro)
   * </ul>
   *
   * <p>This refreshes and returns a cached StatusSignal object.
   *
   * @return DifferentialOutput Status Signal object
   */
  public StatusSignal<Double> getDifferentialOutput() {
    return talonFX.getDifferentialOutput();
  }

  /**
   * The calculated motor output for differential followers.
   *
   * <p>This is a torque request when using the TorqueCurrentFOC control output type, and a duty
   * cycle in all other control types.
   *
   * <p>Default Rates:
   *
   * <ul>
   *   <li><b>CAN 2.0:</b> 100.0 Hz
   *   <li><b>CAN FD:</b> 100.0 Hz (TimeSynced with Pro)
   * </ul>
   *
   * <p>This refreshes and returns a cached StatusSignal object.
   *
   * @param refresh Whether to refresh the StatusSignal before returning it; defaults to true
   * @return DifferentialOutput Status Signal object
   */
  public StatusSignal<Double> getDifferentialOutput(boolean refresh) {
    return talonFX.getDifferentialOutput(refresh);
  }

  /**
   * Differential closed loop proportional component
   *
   * <p>The portion of the differential closed loop output that is the proportional to the error.
   * Alternatively, the p-Contribution of the closed loop output.
   *
   * <p>Default Rates:
   *
   * <ul>
   *   <li><b>CAN 2.0:</b> 4.0 Hz
   *   <li><b>CAN FD:</b> 100.0 Hz (TimeSynced with Pro)
   * </ul>
   *
   * <p>This refreshes and returns a cached StatusSignal object.
   *
   * @return DifferentialClosedLoopProportionalOutput Status Signal object
   */
  public StatusSignal<Double> getDifferentialClosedLoopProportionalOutput() {
    return talonFX.getDifferentialClosedLoopProportionalOutput();
  }

  /**
   * Differential closed loop proportional component
   *
   * <p>The portion of the differential closed loop output that is the proportional to the error.
   * Alternatively, the p-Contribution of the closed loop output.
   *
   * <p>Default Rates:
   *
   * <ul>
   *   <li><b>CAN 2.0:</b> 4.0 Hz
   *   <li><b>CAN FD:</b> 100.0 Hz (TimeSynced with Pro)
   * </ul>
   *
   * <p>This refreshes and returns a cached StatusSignal object.
   *
   * @param refresh Whether to refresh the StatusSignal before returning it; defaults to true
   * @return DifferentialClosedLoopProportionalOutput Status Signal object
   */
  public StatusSignal<Double> getDifferentialClosedLoopProportionalOutput(boolean refresh) {
    return talonFX.getDifferentialClosedLoopProportionalOutput(refresh);
  }

  /**
   * Differential closed loop integrated component
   *
   * <p>The portion of the differential closed loop output that is proportional to the integrated
   * error. Alternatively, the i-Contribution of the closed loop output.
   *
   * <p>Default Rates:
   *
   * <ul>
   *   <li><b>CAN 2.0:</b> 100.0 Hz
   *   <li><b>CAN FD:</b> 100.0 Hz (TimeSynced with Pro)
   * </ul>
   *
   * <p>This refreshes and returns a cached StatusSignal object.
   *
   * @return DifferentialClosedLoopIntegratedOutput Status Signal object
   */
  public StatusSignal<Double> getDifferentialClosedLoopIntegratedOutput() {
    return talonFX.getDifferentialClosedLoopIntegratedOutput();
  }

  /**
   * Differential closed loop integrated component
   *
   * <p>The portion of the differential closed loop output that is proportional to the integrated
   * error. Alternatively, the i-Contribution of the closed loop output.
   *
   * <p>Default Rates:
   *
   * <ul>
   *   <li><b>CAN 2.0:</b> 100.0 Hz
   *   <li><b>CAN FD:</b> 100.0 Hz (TimeSynced with Pro)
   * </ul>
   *
   * <p>This refreshes and returns a cached StatusSignal object.
   *
   * @param refresh Whether to refresh the StatusSignal before returning it; defaults to true
   * @return DifferentialClosedLoopIntegratedOutput Status Signal object
   */
  public StatusSignal<Double> getDifferentialClosedLoopIntegratedOutput(boolean refresh) {
    return talonFX.getDifferentialClosedLoopIntegratedOutput(refresh);
  }

  /**
   * Differential Feedforward passed by the user
   *
   * <p>This is the general feedforward that the user provides for the differential closed loop.
   *
   * <p>Default Rates:
   *
   * <ul>
   *   <li><b>CAN 2.0:</b> 100.0 Hz
   *   <li><b>CAN FD:</b> 100.0 Hz (TimeSynced with Pro)
   * </ul>
   *
   * <p>This refreshes and returns a cached StatusSignal object.
   *
   * @return DifferentialClosedLoopFeedForward Status Signal object
   */
  public StatusSignal<Double> getDifferentialClosedLoopFeedForward() {
    return talonFX.getDifferentialClosedLoopFeedForward();
  }

  /**
   * Differential Feedforward passed by the user
   *
   * <p>This is the general feedforward that the user provides for the differential closed loop.
   *
   * <p>Default Rates:
   *
   * <ul>
   *   <li><b>CAN 2.0:</b> 100.0 Hz
   *   <li><b>CAN FD:</b> 100.0 Hz (TimeSynced with Pro)
   * </ul>
   *
   * <p>This refreshes and returns a cached StatusSignal object.
   *
   * @param refresh Whether to refresh the StatusSignal before returning it; defaults to true
   * @return DifferentialClosedLoopFeedForward Status Signal object
   */
  public StatusSignal<Double> getDifferentialClosedLoopFeedForward(boolean refresh) {
    return talonFX.getDifferentialClosedLoopFeedForward(refresh);
  }

  /**
   * Differential closed loop derivative component
   *
   * <p>The portion of the differential closed loop output that is the proportional to the
   * deriviative the error. Alternatively, the d-Contribution of the closed loop output.
   *
   * <p>Default Rates:
   *
   * <ul>
   *   <li><b>CAN 2.0:</b> 4.0 Hz
   *   <li><b>CAN FD:</b> 100.0 Hz (TimeSynced with Pro)
   * </ul>
   *
   * <p>This refreshes and returns a cached StatusSignal object.
   *
   * @return DifferentialClosedLoopDerivativeOutput Status Signal object
   */
  public StatusSignal<Double> getDifferentialClosedLoopDerivativeOutput() {
    return talonFX.getDifferentialClosedLoopDerivativeOutput();
  }

  /**
   * Differential closed loop derivative component
   *
   * <p>The portion of the differential closed loop output that is the proportional to the
   * deriviative the error. Alternatively, the d-Contribution of the closed loop output.
   *
   * <p>Default Rates:
   *
   * <ul>
   *   <li><b>CAN 2.0:</b> 4.0 Hz
   *   <li><b>CAN FD:</b> 100.0 Hz (TimeSynced with Pro)
   * </ul>
   *
   * <p>This refreshes and returns a cached StatusSignal object.
   *
   * @param refresh Whether to refresh the StatusSignal before returning it; defaults to true
   * @return DifferentialClosedLoopDerivativeOutput Status Signal object
   */
  public StatusSignal<Double> getDifferentialClosedLoopDerivativeOutput(boolean refresh) {
    return talonFX.getDifferentialClosedLoopDerivativeOutput(refresh);
  }

  /**
   * Differential closed loop total output
   *
   * <p>The total output of the differential closed loop output.
   *
   * <p>Default Rates:
   *
   * <ul>
   *   <li><b>CAN 2.0:</b> 4.0 Hz
   *   <li><b>CAN FD:</b> 100.0 Hz (TimeSynced with Pro)
   * </ul>
   *
   * <p>This refreshes and returns a cached StatusSignal object.
   *
   * @return DifferentialClosedLoopOutput Status Signal object
   */
  public StatusSignal<Double> getDifferentialClosedLoopOutput() {
    return talonFX.getDifferentialClosedLoopOutput();
  }

  /**
   * Differential closed loop total output
   *
   * <p>The total output of the differential closed loop output.
   *
   * <p>Default Rates:
   *
   * <ul>
   *   <li><b>CAN 2.0:</b> 4.0 Hz
   *   <li><b>CAN FD:</b> 100.0 Hz (TimeSynced with Pro)
   * </ul>
   *
   * <p>This refreshes and returns a cached StatusSignal object.
   *
   * @param refresh Whether to refresh the StatusSignal before returning it; defaults to true
   * @return DifferentialClosedLoopOutput Status Signal object
   */
  public StatusSignal<Double> getDifferentialClosedLoopOutput(boolean refresh) {
    return talonFX.getDifferentialClosedLoopOutput(refresh);
  }

  /**
   * Value that the differential closed loop is targeting
   *
   * <p>This is the value that the differential closed loop PID controller targets.
   *
   * <p>Default Rates:
   *
   * <ul>
   *   <li><b>CAN 2.0:</b> 4.0 Hz
   *   <li><b>CAN FD:</b> 100.0 Hz (TimeSynced with Pro)
   * </ul>
   *
   * <p>This refreshes and returns a cached StatusSignal object.
   *
   * @return DifferentialClosedLoopReference Status Signal object
   */
  public StatusSignal<Double> getDifferentialClosedLoopReference() {
    return talonFX.getDifferentialClosedLoopReference();
  }

  /**
   * Value that the differential closed loop is targeting
   *
   * <p>This is the value that the differential closed loop PID controller targets.
   *
   * <p>Default Rates:
   *
   * <ul>
   *   <li><b>CAN 2.0:</b> 4.0 Hz
   *   <li><b>CAN FD:</b> 100.0 Hz (TimeSynced with Pro)
   * </ul>
   *
   * <p>This refreshes and returns a cached StatusSignal object.
   *
   * @param refresh Whether to refresh the StatusSignal before returning it; defaults to true
   * @return DifferentialClosedLoopReference Status Signal object
   */
  public StatusSignal<Double> getDifferentialClosedLoopReference(boolean refresh) {
    return talonFX.getDifferentialClosedLoopReference(refresh);
  }

  /**
   * Derivative of the target that the differential closed loop is targeting
   *
   * <p>This is the change in the closed loop reference. This may be used in the feed-forward
   * calculation, the derivative-error, or in application of the signage for kS. Typically, this
   * represents the target velocity during Motion Magic®.
   *
   * <p>Default Rates:
   *
   * <ul>
   *   <li><b>CAN 2.0:</b> 4.0 Hz
   *   <li><b>CAN FD:</b> 100.0 Hz (TimeSynced with Pro)
   * </ul>
   *
   * <p>This refreshes and returns a cached StatusSignal object.
   *
   * @return DifferentialClosedLoopReferenceSlope Status Signal object
   */
  public StatusSignal<Double> getDifferentialClosedLoopReferenceSlope() {
    return talonFX.getDifferentialClosedLoopReferenceSlope();
  }

  /**
   * Derivative of the target that the differential closed loop is targeting
   *
   * <p>This is the change in the closed loop reference. This may be used in the feed-forward
   * calculation, the derivative-error, or in application of the signage for kS. Typically, this
   * represents the target velocity during Motion Magic®.
   *
   * <p>Default Rates:
   *
   * <ul>
   *   <li><b>CAN 2.0:</b> 4.0 Hz
   *   <li><b>CAN FD:</b> 100.0 Hz (TimeSynced with Pro)
   * </ul>
   *
   * <p>This refreshes and returns a cached StatusSignal object.
   *
   * @param refresh Whether to refresh the StatusSignal before returning it; defaults to true
   * @return DifferentialClosedLoopReferenceSlope Status Signal object
   */
  public StatusSignal<Double> getDifferentialClosedLoopReferenceSlope(boolean refresh) {
    return talonFX.getDifferentialClosedLoopReferenceSlope(refresh);
  }

  /**
   * The difference between target differential reference and current measurement
   *
   * <p>This is the value that is treated as the error in the differential PID loop.
   *
   * <p>Default Rates:
   *
   * <ul>
   *   <li><b>CAN 2.0:</b> 4.0 Hz
   *   <li><b>CAN FD:</b> 100.0 Hz (TimeSynced with Pro)
   * </ul>
   *
   * <p>This refreshes and returns a cached StatusSignal object.
   *
   * @return DifferentialClosedLoopError Status Signal object
   */
  public StatusSignal<Double> getDifferentialClosedLoopError() {
    return talonFX.getDifferentialClosedLoopError();
  }

  /**
   * The difference between target differential reference and current measurement
   *
   * <p>This is the value that is treated as the error in the differential PID loop.
   *
   * <p>Default Rates:
   *
   * <ul>
   *   <li><b>CAN 2.0:</b> 4.0 Hz
   *   <li><b>CAN FD:</b> 100.0 Hz (TimeSynced with Pro)
   * </ul>
   *
   * <p>This refreshes and returns a cached StatusSignal object.
   *
   * @param refresh Whether to refresh the StatusSignal before returning it; defaults to true
   * @return DifferentialClosedLoopError Status Signal object
   */
  public StatusSignal<Double> getDifferentialClosedLoopError(boolean refresh) {
    return talonFX.getDifferentialClosedLoopError(refresh);
  }

  /**
   * Request a specified motor duty cycle.
   *
   * <p>This control mode will output a proportion of the supplied voltage which is supplied by the
   * user.
   *
   * <ul>
   *   <li><b>DutyCycleOut Parameters:</b>
   *       <ul>
   *         <li><b>Output:</b> Proportion of supply voltage to apply in fractional units between -1
   *             and +1
   *         <li><b>EnableFOC:</b> Set to true to use FOC commutation (requires Phoenix Pro), which
   *             increases peak power by ~15%. Set to false to use trapezoidal commutation.
   *             <p>FOC improves motor performance by leveraging torque (current) control. However,
   *             this may be inconvenient for applications that require specifying duty cycle or
   *             voltage. CTR-Electronics has developed a hybrid method that combines the
   *             performances gains of FOC while still allowing applications to provide duty cycle
   *             or voltage demand. This not to be confused with simple sinusoidal control or phase
   *             voltage control which lacks the performance gains.
   *         <li><b>OverrideBrakeDurNeutral:</b> Set to true to static-brake the rotor when output
   *             is zero (or within deadband). Set to false to use the NeutralMode configuration
   *             setting (default). This flag exists to provide the fundamental behavior of this
   *             control when output is zero, which is to provide 0V to the motor.
   *         <li><b>LimitForwardMotion:</b> Set to true to force forward limiting. This allows users
   *             to use other limit switch sensors connected to robot controller. This also allows
   *             use of active sensors that require external power.
   *         <li><b>LimitReverseMotion:</b> Set to true to force reverse limiting. This allows users
   *             to use other limit switch sensors connected to robot controller. This also allows
   *             use of active sensors that require external power.
   *         <li><b>IgnoreHardwareLimits:</b> Set to true to ignore hardware limit switches and the
   *             LimitForwardMotion and LimitReverseMotion parameters, instead allowing motion.
   *             <p>This can be useful on mechanisms such as an intake/feeder, where a limit switch
   *             stops motion while intaking but should be ignored when feeding to a shooter.
   *             <p>The hardware limit faults and Forward/ReverseLimit signals will still report the
   *             values of the limit switches regardless of this parameter.
   *         <li><b>UseTimesync:</b> Set to true to delay applying this control request until a
   *             timesync boundary (requires Phoenix Pro and CANivore). This eliminates the impact
   *             of nondeterministic network delays in exchange for a larger but deterministic
   *             control latency.
   *             <p>This requires setting the ControlTimesyncFreqHz config in MotorOutputConfigs.
   *             Additionally, when this is enabled, the UpdateFreqHz of this request should be set
   *             to 0 Hz.
   *       </ul>
   * </ul>
   *
   * @param request Control object to request of the device
   * @return Code response of the request
   */
  public StatusCode setControl(DutyCycleOut request) {
    return talonFX.setControl(request);
  }

  /**
   * Request a specified motor current (field oriented control).
   *
   * <p>This control request will drive the motor to the requested motor (stator) current value.
   * This leverages field oriented control (FOC), which means greater peak power than what is
   * documented. This scales to torque based on Motor's kT constant.
   *
   * <ul>
   *   <li><b>TorqueCurrentFOC Parameters:</b>
   *       <ul>
   *         <li><b>Output:</b> Amount of motor current in Amperes
   *         <li><b>MaxAbsDutyCycle:</b> The maximum absolute motor output that can be applied,
   *             which effectively limits the velocity. For example, 0.50 means no more than 50%
   *             output in either direction. This is useful for preventing the motor from spinning
   *             to its terminal velocity when there is no external torque applied unto the rotor.
   *             Note this is absolute maximum, so the value should be between zero and one.
   *         <li><b>Deadband:</b> Deadband in Amperes. If torque request is within deadband, the
   *             bridge output is neutral. If deadband is set to zero then there is effectively no
   *             deadband. Note if deadband is zero, a free spinning motor will spin for quite a
   *             while as the firmware attempts to hold the motor's bemf. If user expects motor to
   *             cease spinning quickly with a demand of zero, we recommend a deadband of one
   *             Ampere. This value will be converted to an integral value of amps.
   *         <li><b>OverrideCoastDurNeutral:</b> Set to true to coast the rotor when output is zero
   *             (or within deadband). Set to false to use the NeutralMode configuration setting
   *             (default). This flag exists to provide the fundamental behavior of this control
   *             when output is zero, which is to provide 0A (zero torque).
   *         <li><b>LimitForwardMotion:</b> Set to true to force forward limiting. This allows users
   *             to use other limit switch sensors connected to robot controller. This also allows
   *             use of active sensors that require external power.
   *         <li><b>LimitReverseMotion:</b> Set to true to force reverse limiting. This allows users
   *             to use other limit switch sensors connected to robot controller. This also allows
   *             use of active sensors that require external power.
   *         <li><b>IgnoreHardwareLimits:</b> Set to true to ignore hardware limit switches and the
   *             LimitForwardMotion and LimitReverseMotion parameters, instead allowing motion.
   *             <p>This can be useful on mechanisms such as an intake/feeder, where a limit switch
   *             stops motion while intaking but should be ignored when feeding to a shooter.
   *             <p>The hardware limit faults and Forward/ReverseLimit signals will still report the
   *             values of the limit switches regardless of this parameter.
   *         <li><b>UseTimesync:</b> Set to true to delay applying this control request until a
   *             timesync boundary (requires Phoenix Pro and CANivore). This eliminates the impact
   *             of nondeterministic network delays in exchange for a larger but deterministic
   *             control latency.
   *             <p>This requires setting the ControlTimesyncFreqHz config in MotorOutputConfigs.
   *             Additionally, when this is enabled, the UpdateFreqHz of this request should be set
   *             to 0 Hz.
   *       </ul>
   * </ul>
   *
   * @param request Control object to request of the device
   * @return Code response of the request
   */
  public StatusCode setControl(TorqueCurrentFOC request) {
    return talonFX.setControl(request);
  }

  /**
   * Request a specified voltage.
   *
   * <p>This control mode will attempt to apply the specified voltage to the motor. If the supply
   * voltage is below the requested voltage, the motor controller will output the supply voltage.
   *
   * <ul>
   *   <li><b>VoltageOut Parameters:</b>
   *       <ul>
   *         <li><b>Output:</b> Voltage to attempt to drive at
   *         <li><b>EnableFOC:</b> Set to true to use FOC commutation (requires Phoenix Pro), which
   *             increases peak power by ~15%. Set to false to use trapezoidal commutation.
   *             <p>FOC improves motor performance by leveraging torque (current) control. However,
   *             this may be inconvenient for applications that require specifying duty cycle or
   *             voltage. CTR-Electronics has developed a hybrid method that combines the
   *             performances gains of FOC while still allowing applications to provide duty cycle
   *             or voltage demand. This not to be confused with simple sinusoidal control or phase
   *             voltage control which lacks the performance gains.
   *         <li><b>OverrideBrakeDurNeutral:</b> Set to true to static-brake the rotor when output
   *             is zero (or within deadband). Set to false to use the NeutralMode configuration
   *             setting (default). This flag exists to provide the fundamental behavior of this
   *             control when output is zero, which is to provide 0V to the motor.
   *         <li><b>LimitForwardMotion:</b> Set to true to force forward limiting. This allows users
   *             to use other limit switch sensors connected to robot controller. This also allows
   *             use of active sensors that require external power.
   *         <li><b>LimitReverseMotion:</b> Set to true to force reverse limiting. This allows users
   *             to use other limit switch sensors connected to robot controller. This also allows
   *             use of active sensors that require external power.
   *         <li><b>IgnoreHardwareLimits:</b> Set to true to ignore hardware limit switches and the
   *             LimitForwardMotion and LimitReverseMotion parameters, instead allowing motion.
   *             <p>This can be useful on mechanisms such as an intake/feeder, where a limit switch
   *             stops motion while intaking but should be ignored when feeding to a shooter.
   *             <p>The hardware limit faults and Forward/ReverseLimit signals will still report the
   *             values of the limit switches regardless of this parameter.
   *         <li><b>UseTimesync:</b> Set to true to delay applying this control request until a
   *             timesync boundary (requires Phoenix Pro and CANivore). This eliminates the impact
   *             of nondeterministic network delays in exchange for a larger but deterministic
   *             control latency.
   *             <p>This requires setting the ControlTimesyncFreqHz config in MotorOutputConfigs.
   *             Additionally, when this is enabled, the UpdateFreqHz of this request should be set
   *             to 0 Hz.
   *       </ul>
   * </ul>
   *
   * @param request Control object to request of the device
   * @return Code response of the request
   */
  public StatusCode setControl(VoltageOut request) {
    return talonFX.setControl(request);
  }

  /**
   * Request PID to target position with duty cycle feedforward.
   *
   * <p>This control mode will set the motor's position setpoint to the position specified by the
   * user. In addition, it will apply an additional duty cycle as an arbitrary feedforward value.
   *
   * <ul>
   *   <li><b>PositionDutyCycle Parameters:</b>
   *       <ul>
   *         <li><b>Position:</b> Position to drive toward in rotations.
   *         <li><b>Velocity:</b> Velocity to drive toward in rotations per second. This is
   *             typically used for motion profiles generated by the robot program.
   *         <li><b>EnableFOC:</b> Set to true to use FOC commutation (requires Phoenix Pro), which
   *             increases peak power by ~15%. Set to false to use trapezoidal commutation.
   *             <p>FOC improves motor performance by leveraging torque (current) control. However,
   *             this may be inconvenient for applications that require specifying duty cycle or
   *             voltage. CTR-Electronics has developed a hybrid method that combines the
   *             performances gains of FOC while still allowing applications to provide duty cycle
   *             or voltage demand. This not to be confused with simple sinusoidal control or phase
   *             voltage control which lacks the performance gains.
   *         <li><b>FeedForward:</b> Feedforward to apply in fractional units between -1 and +1.
   *         <li><b>Slot:</b> Select which gains are applied by selecting the slot. Use the
   *             configuration api to set the gain values for the selected slot before enabling this
   *             feature. Slot must be within [0,2].
   *         <li><b>OverrideBrakeDurNeutral:</b> Set to true to static-brake the rotor when output
   *             is zero (or within deadband). Set to false to use the NeutralMode configuration
   *             setting (default). This flag exists to provide the fundamental behavior of this
   *             control when output is zero, which is to provide 0V to the motor.
   *         <li><b>LimitForwardMotion:</b> Set to true to force forward limiting. This allows users
   *             to use other limit switch sensors connected to robot controller. This also allows
   *             use of active sensors that require external power.
   *         <li><b>LimitReverseMotion:</b> Set to true to force reverse limiting. This allows users
   *             to use other limit switch sensors connected to robot controller. This also allows
   *             use of active sensors that require external power.
   *         <li><b>IgnoreHardwareLimits:</b> Set to true to ignore hardware limit switches and the
   *             LimitForwardMotion and LimitReverseMotion parameters, instead allowing motion.
   *             <p>This can be useful on mechanisms such as an intake/feeder, where a limit switch
   *             stops motion while intaking but should be ignored when feeding to a shooter.
   *             <p>The hardware limit faults and Forward/ReverseLimit signals will still report the
   *             values of the limit switches regardless of this parameter.
   *         <li><b>UseTimesync:</b> Set to true to delay applying this control request until a
   *             timesync boundary (requires Phoenix Pro and CANivore). This eliminates the impact
   *             of nondeterministic network delays in exchange for a larger but deterministic
   *             control latency.
   *             <p>This requires setting the ControlTimesyncFreqHz config in MotorOutputConfigs.
   *             Additionally, when this is enabled, the UpdateFreqHz of this request should be set
   *             to 0 Hz.
   *       </ul>
   * </ul>
   *
   * @param request Control object to request of the device
   * @return Code response of the request
   */
  public StatusCode setControl(PositionDutyCycle request) {
    return talonFX.setControl(request);
  }

  /**
   * Request PID to target position with voltage feedforward
   *
   * <p>This control mode will set the motor's position setpoint to the position specified by the
   * user. In addition, it will apply an additional voltage as an arbitrary feedforward value.
   *
   * <ul>
   *   <li><b>PositionVoltage Parameters:</b>
   *       <ul>
   *         <li><b>Position:</b> Position to drive toward in rotations.
   *         <li><b>Velocity:</b> Velocity to drive toward in rotations per second. This is
   *             typically used for motion profiles generated by the robot program.
   *         <li><b>EnableFOC:</b> Set to true to use FOC commutation (requires Phoenix Pro), which
   *             increases peak power by ~15%. Set to false to use trapezoidal commutation.
   *             <p>FOC improves motor performance by leveraging torque (current) control. However,
   *             this may be inconvenient for applications that require specifying duty cycle or
   *             voltage. CTR-Electronics has developed a hybrid method that combines the
   *             performances gains of FOC while still allowing applications to provide duty cycle
   *             or voltage demand. This not to be confused with simple sinusoidal control or phase
   *             voltage control which lacks the performance gains.
   *         <li><b>FeedForward:</b> Feedforward to apply in volts
   *         <li><b>Slot:</b> Select which gains are applied by selecting the slot. Use the
   *             configuration api to set the gain values for the selected slot before enabling this
   *             feature. Slot must be within [0,2].
   *         <li><b>OverrideBrakeDurNeutral:</b> Set to true to static-brake the rotor when output
   *             is zero (or within deadband). Set to false to use the NeutralMode configuration
   *             setting (default). This flag exists to provide the fundamental behavior of this
   *             control when output is zero, which is to provide 0V to the motor.
   *         <li><b>LimitForwardMotion:</b> Set to true to force forward limiting. This allows users
   *             to use other limit switch sensors connected to robot controller. This also allows
   *             use of active sensors that require external power.
   *         <li><b>LimitReverseMotion:</b> Set to true to force reverse limiting. This allows users
   *             to use other limit switch sensors connected to robot controller. This also allows
   *             use of active sensors that require external power.
   *         <li><b>IgnoreHardwareLimits:</b> Set to true to ignore hardware limit switches and the
   *             LimitForwardMotion and LimitReverseMotion parameters, instead allowing motion.
   *             <p>This can be useful on mechanisms such as an intake/feeder, where a limit switch
   *             stops motion while intaking but should be ignored when feeding to a shooter.
   *             <p>The hardware limit faults and Forward/ReverseLimit signals will still report the
   *             values of the limit switches regardless of this parameter.
   *         <li><b>UseTimesync:</b> Set to true to delay applying this control request until a
   *             timesync boundary (requires Phoenix Pro and CANivore). This eliminates the impact
   *             of nondeterministic network delays in exchange for a larger but deterministic
   *             control latency.
   *             <p>This requires setting the ControlTimesyncFreqHz config in MotorOutputConfigs.
   *             Additionally, when this is enabled, the UpdateFreqHz of this request should be set
   *             to 0 Hz.
   *       </ul>
   * </ul>
   *
   * @param request Control object to request of the device
   * @return Code response of the request
   */
  public StatusCode setControl(PositionVoltage request) {
    return talonFX.setControl(request);
  }

  /**
   * Request PID to target position with torque current feedforward.
   *
   * <p>This control mode will set the motor's position setpoint to the position specified by the
   * user. In addition, it will apply an additional torque current as an arbitrary feedforward
   * value.
   *
   * <ul>
   *   <li><b>PositionTorqueCurrentFOC Parameters:</b>
   *       <ul>
   *         <li><b>Position:</b> Position to drive toward in rotations.
   *         <li><b>Velocity:</b> Velocity to drive toward in rotations per second. This is
   *             typically used for motion profiles generated by the robot program.
   *         <li><b>FeedForward:</b> Feedforward to apply in torque current in Amperes. User can use
   *             motor's kT to scale Newton-meter to Amperes.
   *         <li><b>Slot:</b> Select which gains are applied by selecting the slot. Use the
   *             configuration api to set the gain values for the selected slot before enabling this
   *             feature. Slot must be within [0,2].
   *         <li><b>OverrideCoastDurNeutral:</b> Set to true to coast the rotor when output is zero
   *             (or within deadband). Set to false to use the NeutralMode configuration setting
   *             (default). This flag exists to provide the fundamental behavior of this control
   *             when output is zero, which is to provide 0A (zero torque).
   *         <li><b>LimitForwardMotion:</b> Set to true to force forward limiting. This allows users
   *             to use other limit switch sensors connected to robot controller. This also allows
   *             use of active sensors that require external power.
   *         <li><b>LimitReverseMotion:</b> Set to true to force reverse limiting. This allows users
   *             to use other limit switch sensors connected to robot controller. This also allows
   *             use of active sensors that require external power.
   *         <li><b>IgnoreHardwareLimits:</b> Set to true to ignore hardware limit switches and the
   *             LimitForwardMotion and LimitReverseMotion parameters, instead allowing motion.
   *             <p>This can be useful on mechanisms such as an intake/feeder, where a limit switch
   *             stops motion while intaking but should be ignored when feeding to a shooter.
   *             <p>The hardware limit faults and Forward/ReverseLimit signals will still report the
   *             values of the limit switches regardless of this parameter.
   *         <li><b>UseTimesync:</b> Set to true to delay applying this control request until a
   *             timesync boundary (requires Phoenix Pro and CANivore). This eliminates the impact
   *             of nondeterministic network delays in exchange for a larger but deterministic
   *             control latency.
   *             <p>This requires setting the ControlTimesyncFreqHz config in MotorOutputConfigs.
   *             Additionally, when this is enabled, the UpdateFreqHz of this request should be set
   *             to 0 Hz.
   *       </ul>
   * </ul>
   *
   * @param request Control object to request of the device
   * @return Code response of the request
   */
  public StatusCode setControl(PositionTorqueCurrentFOC request) {
    return talonFX.setControl(request);
  }

  /**
   * Request PID to target velocity with duty cycle feedforward.
   *
   * <p>This control mode will set the motor's velocity setpoint to the velocity specified by the
   * user. In addition, it will apply an additional voltage as an arbitrary feedforward value.
   *
   * <ul>
   *   <li><b>VelocityDutyCycle Parameters:</b>
   *       <ul>
   *         <li><b>Velocity:</b> Velocity to drive toward in rotations per second.
   *         <li><b>Acceleration:</b> Acceleration to drive toward in rotations per second squared.
   *             This is typically used for motion profiles generated by the robot program.
   *         <li><b>EnableFOC:</b> Set to true to use FOC commutation (requires Phoenix Pro), which
   *             increases peak power by ~15%. Set to false to use trapezoidal commutation.
   *             <p>FOC improves motor performance by leveraging torque (current) control. However,
   *             this may be inconvenient for applications that require specifying duty cycle or
   *             voltage. CTR-Electronics has developed a hybrid method that combines the
   *             performances gains of FOC while still allowing applications to provide duty cycle
   *             or voltage demand. This not to be confused with simple sinusoidal control or phase
   *             voltage control which lacks the performance gains.
   *         <li><b>FeedForward:</b> Feedforward to apply in fractional units between -1 and +1.
   *         <li><b>Slot:</b> Select which gains are applied by selecting the slot. Use the
   *             configuration api to set the gain values for the selected slot before enabling this
   *             feature. Slot must be within [0,2].
   *         <li><b>OverrideBrakeDurNeutral:</b> Set to true to static-brake the rotor when output
   *             is zero (or within deadband). Set to false to use the NeutralMode configuration
   *             setting (default). This flag exists to provide the fundamental behavior of this
   *             control when output is zero, which is to provide 0V to the motor.
   *         <li><b>LimitForwardMotion:</b> Set to true to force forward limiting. This allows users
   *             to use other limit switch sensors connected to robot controller. This also allows
   *             use of active sensors that require external power.
   *         <li><b>LimitReverseMotion:</b> Set to true to force reverse limiting. This allows users
   *             to use other limit switch sensors connected to robot controller. This also allows
   *             use of active sensors that require external power.
   *         <li><b>IgnoreHardwareLimits:</b> Set to true to ignore hardware limit switches and the
   *             LimitForwardMotion and LimitReverseMotion parameters, instead allowing motion.
   *             <p>This can be useful on mechanisms such as an intake/feeder, where a limit switch
   *             stops motion while intaking but should be ignored when feeding to a shooter.
   *             <p>The hardware limit faults and Forward/ReverseLimit signals will still report the
   *             values of the limit switches regardless of this parameter.
   *         <li><b>UseTimesync:</b> Set to true to delay applying this control request until a
   *             timesync boundary (requires Phoenix Pro and CANivore). This eliminates the impact
   *             of nondeterministic network delays in exchange for a larger but deterministic
   *             control latency.
   *             <p>This requires setting the ControlTimesyncFreqHz config in MotorOutputConfigs.
   *             Additionally, when this is enabled, the UpdateFreqHz of this request should be set
   *             to 0 Hz.
   *       </ul>
   * </ul>
   *
   * @param request Control object to request of the device
   * @return Code response of the request
   */
  public StatusCode setControl(VelocityDutyCycle request) {
    return talonFX.setControl(request);
  }

  /**
   * Request PID to target velocity with voltage feedforward.
   *
   * <p>This control mode will set the motor's velocity setpoint to the velocity specified by the
   * user. In addition, it will apply an additional voltage as an arbitrary feedforward value.
   *
   * <ul>
   *   <li><b>VelocityVoltage Parameters:</b>
   *       <ul>
   *         <li><b>Velocity:</b> Velocity to drive toward in rotations per second.
   *         <li><b>Acceleration:</b> Acceleration to drive toward in rotations per second squared.
   *             This is typically used for motion profiles generated by the robot program.
   *         <li><b>EnableFOC:</b> Set to true to use FOC commutation (requires Phoenix Pro), which
   *             increases peak power by ~15%. Set to false to use trapezoidal commutation.
   *             <p>FOC improves motor performance by leveraging torque (current) control. However,
   *             this may be inconvenient for applications that require specifying duty cycle or
   *             voltage. CTR-Electronics has developed a hybrid method that combines the
   *             performances gains of FOC while still allowing applications to provide duty cycle
   *             or voltage demand. This not to be confused with simple sinusoidal control or phase
   *             voltage control which lacks the performance gains.
   *         <li><b>FeedForward:</b> Feedforward to apply in volts
   *         <li><b>Slot:</b> Select which gains are applied by selecting the slot. Use the
   *             configuration api to set the gain values for the selected slot before enabling this
   *             feature. Slot must be within [0,2].
   *         <li><b>OverrideBrakeDurNeutral:</b> Set to true to static-brake the rotor when output
   *             is zero (or within deadband). Set to false to use the NeutralMode configuration
   *             setting (default). This flag exists to provide the fundamental behavior of this
   *             control when output is zero, which is to provide 0V to the motor.
   *         <li><b>LimitForwardMotion:</b> Set to true to force forward limiting. This allows users
   *             to use other limit switch sensors connected to robot controller. This also allows
   *             use of active sensors that require external power.
   *         <li><b>LimitReverseMotion:</b> Set to true to force reverse limiting. This allows users
   *             to use other limit switch sensors connected to robot controller. This also allows
   *             use of active sensors that require external power.
   *         <li><b>IgnoreHardwareLimits:</b> Set to true to ignore hardware limit switches and the
   *             LimitForwardMotion and LimitReverseMotion parameters, instead allowing motion.
   *             <p>This can be useful on mechanisms such as an intake/feeder, where a limit switch
   *             stops motion while intaking but should be ignored when feeding to a shooter.
   *             <p>The hardware limit faults and Forward/ReverseLimit signals will still report the
   *             values of the limit switches regardless of this parameter.
   *         <li><b>UseTimesync:</b> Set to true to delay applying this control request until a
   *             timesync boundary (requires Phoenix Pro and CANivore). This eliminates the impact
   *             of nondeterministic network delays in exchange for a larger but deterministic
   *             control latency.
   *             <p>This requires setting the ControlTimesyncFreqHz config in MotorOutputConfigs.
   *             Additionally, when this is enabled, the UpdateFreqHz of this request should be set
   *             to 0 Hz.
   *       </ul>
   * </ul>
   *
   * @param request Control object to request of the device
   * @return Code response of the request
   */
  public StatusCode setControl(VelocityVoltage request) {
    return talonFX.setControl(request);
  }

  /**
   * Request PID to target velocity with torque current feedforward.
   *
   * <p>This control mode will set the motor's velocity setpoint to the velocity specified by the
   * user. In addition, it will apply an additional torque current as an arbitrary feedforward
   * value.
   *
   * <ul>
   *   <li><b>VelocityTorqueCurrentFOC Parameters:</b>
   *       <ul>
   *         <li><b>Velocity:</b> Velocity to drive toward in rotations per second.
   *         <li><b>Acceleration:</b> Acceleration to drive toward in rotations per second squared.
   *             This is typically used for motion profiles generated by the robot program.
   *         <li><b>FeedForward:</b> Feedforward to apply in torque current in Amperes. User can use
   *             motor's kT to scale Newton-meter to Amperes.
   *         <li><b>Slot:</b> Select which gains are applied by selecting the slot. Use the
   *             configuration api to set the gain values for the selected slot before enabling this
   *             feature. Slot must be within [0,2].
   *         <li><b>OverrideCoastDurNeutral:</b> Set to true to coast the rotor when output is zero
   *             (or within deadband). Set to false to use the NeutralMode configuration setting
   *             (default). This flag exists to provide the fundamental behavior of this control
   *             when output is zero, which is to provide 0A (zero torque).
   *         <li><b>LimitForwardMotion:</b> Set to true to force forward limiting. This allows users
   *             to use other limit switch sensors connected to robot controller. This also allows
   *             use of active sensors that require external power.
   *         <li><b>LimitReverseMotion:</b> Set to true to force reverse limiting. This allows users
   *             to use other limit switch sensors connected to robot controller. This also allows
   *             use of active sensors that require external power.
   *         <li><b>IgnoreHardwareLimits:</b> Set to true to ignore hardware limit switches and the
   *             LimitForwardMotion and LimitReverseMotion parameters, instead allowing motion.
   *             <p>This can be useful on mechanisms such as an intake/feeder, where a limit switch
   *             stops motion while intaking but should be ignored when feeding to a shooter.
   *             <p>The hardware limit faults and Forward/ReverseLimit signals will still report the
   *             values of the limit switches regardless of this parameter.
   *         <li><b>UseTimesync:</b> Set to true to delay applying this control request until a
   *             timesync boundary (requires Phoenix Pro and CANivore). This eliminates the impact
   *             of nondeterministic network delays in exchange for a larger but deterministic
   *             control latency.
   *             <p>This requires setting the ControlTimesyncFreqHz config in MotorOutputConfigs.
   *             Additionally, when this is enabled, the UpdateFreqHz of this request should be set
   *             to 0 Hz.
   *       </ul>
   * </ul>
   *
   * @param request Control object to request of the device
   * @return Code response of the request
   */
  public StatusCode setControl(VelocityTorqueCurrentFOC request) {
    return talonFX.setControl(request);
  }

  /**
   * Requests Motion Magic® to target a final position using a motion profile. Users can optionally
   * provide a duty cycle feedforward.
   *
   * <p>Motion Magic® produces a motion profile in real-time while attempting to honor the Cruise
   * Velocity, Acceleration, and (optional) Jerk specified via the Motion Magic® configuration
   * values. This control mode does not use the Expo_kV or Expo_kA configs.
   *
   * <p>Target position can be changed on-the-fly and Motion Magic® will do its best to adjust the
   * profile. This control mode is duty cycle based, so relevant closed-loop gains will use
   * fractional duty cycle for the numerator: +1.0 represents full forward output.
   *
   * <ul>
   *   <li><b>MotionMagicDutyCycle Parameters:</b>
   *       <ul>
   *         <li><b>Position:</b> Position to drive toward in rotations.
   *         <li><b>EnableFOC:</b> Set to true to use FOC commutation (requires Phoenix Pro), which
   *             increases peak power by ~15%. Set to false to use trapezoidal commutation.
   *             <p>FOC improves motor performance by leveraging torque (current) control. However,
   *             this may be inconvenient for applications that require specifying duty cycle or
   *             voltage. CTR-Electronics has developed a hybrid method that combines the
   *             performances gains of FOC while still allowing applications to provide duty cycle
   *             or voltage demand. This not to be confused with simple sinusoidal control or phase
   *             voltage control which lacks the performance gains.
   *         <li><b>FeedForward:</b> Feedforward to apply in fractional units between -1 and +1.
   *         <li><b>Slot:</b> Select which gains are applied by selecting the slot. Use the
   *             configuration api to set the gain values for the selected slot before enabling this
   *             feature. Slot must be within [0,2].
   *         <li><b>OverrideBrakeDurNeutral:</b> Set to true to static-brake the rotor when output
   *             is zero (or within deadband). Set to false to use the NeutralMode configuration
   *             setting (default). This flag exists to provide the fundamental behavior of this
   *             control when output is zero, which is to provide 0V to the motor.
   *         <li><b>LimitForwardMotion:</b> Set to true to force forward limiting. This allows users
   *             to use other limit switch sensors connected to robot controller. This also allows
   *             use of active sensors that require external power.
   *         <li><b>LimitReverseMotion:</b> Set to true to force reverse limiting. This allows users
   *             to use other limit switch sensors connected to robot controller. This also allows
   *             use of active sensors that require external power.
   *         <li><b>IgnoreHardwareLimits:</b> Set to true to ignore hardware limit switches and the
   *             LimitForwardMotion and LimitReverseMotion parameters, instead allowing motion.
   *             <p>This can be useful on mechanisms such as an intake/feeder, where a limit switch
   *             stops motion while intaking but should be ignored when feeding to a shooter.
   *             <p>The hardware limit faults and Forward/ReverseLimit signals will still report the
   *             values of the limit switches regardless of this parameter.
   *         <li><b>UseTimesync:</b> Set to true to delay applying this control request until a
   *             timesync boundary (requires Phoenix Pro and CANivore). This eliminates the impact
   *             of nondeterministic network delays in exchange for a larger but deterministic
   *             control latency.
   *             <p>This requires setting the ControlTimesyncFreqHz config in MotorOutputConfigs.
   *             Additionally, when this is enabled, the UpdateFreqHz of this request should be set
   *             to 0 Hz.
   *       </ul>
   * </ul>
   *
   * @param request Control object to request of the device
   * @return Code response of the request
   */
  public StatusCode setControl(MotionMagicDutyCycle request) {
    return talonFX.setControl(request);
  }

  /**
   * Requests Motion Magic® to target a final position using a motion profile. Users can optionally
   * provide a voltage feedforward.
   *
   * <p>Motion Magic® produces a motion profile in real-time while attempting to honor the Cruise
   * Velocity, Acceleration, and (optional) Jerk specified via the Motion Magic® configuration
   * values. This control mode does not use the Expo_kV or Expo_kA configs.
   *
   * <p>Target position can be changed on-the-fly and Motion Magic® will do its best to adjust the
   * profile. This control mode is voltage-based, so relevant closed-loop gains will use Volts for
   * the numerator.
   *
   * <ul>
   *   <li><b>MotionMagicVoltage Parameters:</b>
   *       <ul>
   *         <li><b>Position:</b> Position to drive toward in rotations.
   *         <li><b>EnableFOC:</b> Set to true to use FOC commutation (requires Phoenix Pro), which
   *             increases peak power by ~15%. Set to false to use trapezoidal commutation.
   *             <p>FOC improves motor performance by leveraging torque (current) control. However,
   *             this may be inconvenient for applications that require specifying duty cycle or
   *             voltage. CTR-Electronics has developed a hybrid method that combines the
   *             performances gains of FOC while still allowing applications to provide duty cycle
   *             or voltage demand. This not to be confused with simple sinusoidal control or phase
   *             voltage control which lacks the performance gains.
   *         <li><b>FeedForward:</b> Feedforward to apply in volts
   *         <li><b>Slot:</b> Select which gains are applied by selecting the slot. Use the
   *             configuration api to set the gain values for the selected slot before enabling this
   *             feature. Slot must be within [0,2].
   *         <li><b>OverrideBrakeDurNeutral:</b> Set to true to static-brake the rotor when output
   *             is zero (or within deadband). Set to false to use the NeutralMode configuration
   *             setting (default). This flag exists to provide the fundamental behavior of this
   *             control when output is zero, which is to provide 0V to the motor.
   *         <li><b>LimitForwardMotion:</b> Set to true to force forward limiting. This allows users
   *             to use other limit switch sensors connected to robot controller. This also allows
   *             use of active sensors that require external power.
   *         <li><b>LimitReverseMotion:</b> Set to true to force reverse limiting. This allows users
   *             to use other limit switch sensors connected to robot controller. This also allows
   *             use of active sensors that require external power.
   *         <li><b>IgnoreHardwareLimits:</b> Set to true to ignore hardware limit switches and the
   *             LimitForwardMotion and LimitReverseMotion parameters, instead allowing motion.
   *             <p>This can be useful on mechanisms such as an intake/feeder, where a limit switch
   *             stops motion while intaking but should be ignored when feeding to a shooter.
   *             <p>The hardware limit faults and Forward/ReverseLimit signals will still report the
   *             values of the limit switches regardless of this parameter.
   *         <li><b>UseTimesync:</b> Set to true to delay applying this control request until a
   *             timesync boundary (requires Phoenix Pro and CANivore). This eliminates the impact
   *             of nondeterministic network delays in exchange for a larger but deterministic
   *             control latency.
   *             <p>This requires setting the ControlTimesyncFreqHz config in MotorOutputConfigs.
   *             Additionally, when this is enabled, the UpdateFreqHz of this request should be set
   *             to 0 Hz.
   *       </ul>
   * </ul>
   *
   * @param request Control object to request of the device
   * @return Code response of the request
   */
  public StatusCode setControl(MotionMagicVoltage request) {
    return talonFX.setControl(request);
  }

  /**
   * Requests Motion Magic® to target a final position using a motion profile. Users can optionally
   * provide a torque current feedforward.
   *
   * <p>Motion Magic® produces a motion profile in real-time while attempting to honor the Cruise
   * Velocity, Acceleration, and (optional) Jerk specified via the Motion Magic® configuration
   * values. This control mode does not use the Expo_kV or Expo_kA configs.
   *
   * <p>Target position can be changed on-the-fly and Motion Magic® will do its best to adjust the
   * profile. This control mode is based on torque current, so relevant closed-loop gains will use
   * Amperes for the numerator.
   *
   * <ul>
   *   <li><b>MotionMagicTorqueCurrentFOC Parameters:</b>
   *       <ul>
   *         <li><b>Position:</b> Position to drive toward in rotations.
   *         <li><b>FeedForward:</b> Feedforward to apply in torque current in Amperes. User can use
   *             motor's kT to scale Newton-meter to Amperes.
   *         <li><b>Slot:</b> Select which gains are applied by selecting the slot. Use the
   *             configuration api to set the gain values for the selected slot before enabling this
   *             feature. Slot must be within [0,2].
   *         <li><b>OverrideCoastDurNeutral:</b> Set to true to coast the rotor when output is zero
   *             (or within deadband). Set to false to use the NeutralMode configuration setting
   *             (default). This flag exists to provide the fundamental behavior of this control
   *             when output is zero, which is to provide 0A (zero torque).
   *         <li><b>LimitForwardMotion:</b> Set to true to force forward limiting. This allows users
   *             to use other limit switch sensors connected to robot controller. This also allows
   *             use of active sensors that require external power.
   *         <li><b>LimitReverseMotion:</b> Set to true to force reverse limiting. This allows users
   *             to use other limit switch sensors connected to robot controller. This also allows
   *             use of active sensors that require external power.
   *         <li><b>IgnoreHardwareLimits:</b> Set to true to ignore hardware limit switches and the
   *             LimitForwardMotion and LimitReverseMotion parameters, instead allowing motion.
   *             <p>This can be useful on mechanisms such as an intake/feeder, where a limit switch
   *             stops motion while intaking but should be ignored when feeding to a shooter.
   *             <p>The hardware limit faults and Forward/ReverseLimit signals will still report the
   *             values of the limit switches regardless of this parameter.
   *         <li><b>UseTimesync:</b> Set to true to delay applying this control request until a
   *             timesync boundary (requires Phoenix Pro and CANivore). This eliminates the impact
   *             of nondeterministic network delays in exchange for a larger but deterministic
   *             control latency.
   *             <p>This requires setting the ControlTimesyncFreqHz config in MotorOutputConfigs.
   *             Additionally, when this is enabled, the UpdateFreqHz of this request should be set
   *             to 0 Hz.
   *       </ul>
   * </ul>
   *
   * @param request Control object to request of the device
   * @return Code response of the request
   */
  public StatusCode setControl(MotionMagicTorqueCurrentFOC request) {
    return talonFX.setControl(request);
  }

  /**
   * Request a specified motor duty cycle with a differential position closed-loop.
   *
   * <p>This control mode will output a proportion of the supplied voltage which is supplied by the
   * user. It will also set the motor's differential position setpoint to the specified position.
   *
   * <ul>
   *   <li><b>DifferentialDutyCycle Parameters:</b>
   *       <ul>
   *         <li><b>TargetOutput:</b> Proportion of supply voltage to apply in fractional units
   *             between -1 and +1
   *         <li><b>DifferentialPosition:</b> Differential position to drive towards in rotations
   *         <li><b>EnableFOC:</b> Set to true to use FOC commutation (requires Phoenix Pro), which
   *             increases peak power by ~15%. Set to false to use trapezoidal commutation.
   *             <p>FOC improves motor performance by leveraging torque (current) control. However,
   *             this may be inconvenient for applications that require specifying duty cycle or
   *             voltage. CTR-Electronics has developed a hybrid method that combines the
   *             performances gains of FOC while still allowing applications to provide duty cycle
   *             or voltage demand. This not to be confused with simple sinusoidal control or phase
   *             voltage control which lacks the performance gains.
   *         <li><b>DifferentialSlot:</b> Select which gains are applied to the differential
   *             controller by selecting the slot. Use the configuration api to set the gain values
   *             for the selected slot before enabling this feature. Slot must be within [0,2].
   *         <li><b>OverrideBrakeDurNeutral:</b> Set to true to static-brake the rotor when output
   *             is zero (or within deadband). Set to false to use the NeutralMode configuration
   *             setting (default). This flag exists to provide the fundamental behavior of this
   *             control when output is zero, which is to provide 0V to the motor.
   *         <li><b>LimitForwardMotion:</b> Set to true to force forward limiting. This allows users
   *             to use other limit switch sensors connected to robot controller. This also allows
   *             use of active sensors that require external power.
   *         <li><b>LimitReverseMotion:</b> Set to true to force reverse limiting. This allows users
   *             to use other limit switch sensors connected to robot controller. This also allows
   *             use of active sensors that require external power.
   *         <li><b>IgnoreHardwareLimits:</b> Set to true to ignore hardware limit switches and the
   *             LimitForwardMotion and LimitReverseMotion parameters, instead allowing motion.
   *             <p>This can be useful on mechanisms such as an intake/feeder, where a limit switch
   *             stops motion while intaking but should be ignored when feeding to a shooter.
   *             <p>The hardware limit faults and Forward/ReverseLimit signals will still report the
   *             values of the limit switches regardless of this parameter.
   *         <li><b>UseTimesync:</b> Set to true to delay applying this control request until a
   *             timesync boundary (requires Phoenix Pro and CANivore). This eliminates the impact
   *             of nondeterministic network delays in exchange for a larger but deterministic
   *             control latency.
   *             <p>This requires setting the ControlTimesyncFreqHz config in MotorOutputConfigs.
   *             Additionally, when this is enabled, the UpdateFreqHz of this request should be set
   *             to 0 Hz.
   *       </ul>
   * </ul>
   *
   * @param request Control object to request of the device
   * @return Code response of the request
   */
  public StatusCode setControl(DifferentialDutyCycle request) {
    return talonFX.setControl(request);
  }

  /**
   * Request a specified voltage with a differential position closed-loop.
   *
   * <p>This control mode will attempt to apply the specified voltage to the motor. If the supply
   * voltage is below the requested voltage, the motor controller will output the supply voltage. It
   * will also set the motor's differential position setpoint to the specified position.
   *
   * <ul>
   *   <li><b>DifferentialVoltage Parameters:</b>
   *       <ul>
   *         <li><b>TargetOutput:</b> Voltage to attempt to drive at
   *         <li><b>DifferentialPosition:</b> Differential position to drive towards in rotations
   *         <li><b>EnableFOC:</b> Set to true to use FOC commutation (requires Phoenix Pro), which
   *             increases peak power by ~15%. Set to false to use trapezoidal commutation.
   *             <p>FOC improves motor performance by leveraging torque (current) control. However,
   *             this may be inconvenient for applications that require specifying duty cycle or
   *             voltage. CTR-Electronics has developed a hybrid method that combines the
   *             performances gains of FOC while still allowing applications to provide duty cycle
   *             or voltage demand. This not to be confused with simple sinusoidal control or phase
   *             voltage control which lacks the performance gains.
   *         <li><b>DifferentialSlot:</b> Select which gains are applied to the differential
   *             controller by selecting the slot. Use the configuration api to set the gain values
   *             for the selected slot before enabling this feature. Slot must be within [0,2].
   *         <li><b>OverrideBrakeDurNeutral:</b> Set to true to static-brake the rotor when output
   *             is zero (or within deadband). Set to false to use the NeutralMode configuration
   *             setting (default). This flag exists to provide the fundamental behavior of this
   *             control when output is zero, which is to provide 0V to the motor.
   *         <li><b>LimitForwardMotion:</b> Set to true to force forward limiting. This allows users
   *             to use other limit switch sensors connected to robot controller. This also allows
   *             use of active sensors that require external power.
   *         <li><b>LimitReverseMotion:</b> Set to true to force reverse limiting. This allows users
   *             to use other limit switch sensors connected to robot controller. This also allows
   *             use of active sensors that require external power.
   *         <li><b>IgnoreHardwareLimits:</b> Set to true to ignore hardware limit switches and the
   *             LimitForwardMotion and LimitReverseMotion parameters, instead allowing motion.
   *             <p>This can be useful on mechanisms such as an intake/feeder, where a limit switch
   *             stops motion while intaking but should be ignored when feeding to a shooter.
   *             <p>The hardware limit faults and Forward/ReverseLimit signals will still report the
   *             values of the limit switches regardless of this parameter.
   *         <li><b>UseTimesync:</b> Set to true to delay applying this control request until a
   *             timesync boundary (requires Phoenix Pro and CANivore). This eliminates the impact
   *             of nondeterministic network delays in exchange for a larger but deterministic
   *             control latency.
   *             <p>This requires setting the ControlTimesyncFreqHz config in MotorOutputConfigs.
   *             Additionally, when this is enabled, the UpdateFreqHz of this request should be set
   *             to 0 Hz.
   *       </ul>
   * </ul>
   *
   * @param request Control object to request of the device
   * @return Code response of the request
   */
  public StatusCode setControl(DifferentialVoltage request) {
    return talonFX.setControl(request);
  }

  /**
   * Request PID to target position with a differential position setpoint.
   *
   * <p>This control mode will set the motor's position setpoint to the position specified by the
   * user. It will also set the motor's differential position setpoint to the specified position.
   *
   * <ul>
   *   <li><b>DifferentialPositionDutyCycle Parameters:</b>
   *       <ul>
   *         <li><b>TargetPosition:</b> Average position to drive toward in rotations.
   *         <li><b>DifferentialPosition:</b> Differential position to drive toward in rotations.
   *         <li><b>EnableFOC:</b> Set to true to use FOC commutation (requires Phoenix Pro), which
   *             increases peak power by ~15%. Set to false to use trapezoidal commutation.
   *             <p>FOC improves motor performance by leveraging torque (current) control. However,
   *             this may be inconvenient for applications that require specifying duty cycle or
   *             voltage. CTR-Electronics has developed a hybrid method that combines the
   *             performances gains of FOC while still allowing applications to provide duty cycle
   *             or voltage demand. This not to be confused with simple sinusoidal control or phase
   *             voltage control which lacks the performance gains.
   *         <li><b>TargetSlot:</b> Select which gains are applied to the primary controller by
   *             selecting the slot. Use the configuration api to set the gain values for the
   *             selected slot before enabling this feature. Slot must be within [0,2].
   *         <li><b>DifferentialSlot:</b> Select which gains are applied to the differential
   *             controller by selecting the slot. Use the configuration api to set the gain values
   *             for the selected slot before enabling this feature. Slot must be within [0,2].
   *         <li><b>OverrideBrakeDurNeutral:</b> Set to true to static-brake the rotor when output
   *             is zero (or within deadband). Set to false to use the NeutralMode configuration
   *             setting (default). This flag exists to provide the fundamental behavior of this
   *             control when output is zero, which is to provide 0V to the motor.
   *         <li><b>LimitForwardMotion:</b> Set to true to force forward limiting. This allows users
   *             to use other limit switch sensors connected to robot controller. This also allows
   *             use of active sensors that require external power.
   *         <li><b>LimitReverseMotion:</b> Set to true to force reverse limiting. This allows users
   *             to use other limit switch sensors connected to robot controller. This also allows
   *             use of active sensors that require external power.
   *         <li><b>IgnoreHardwareLimits:</b> Set to true to ignore hardware limit switches and the
   *             LimitForwardMotion and LimitReverseMotion parameters, instead allowing motion.
   *             <p>This can be useful on mechanisms such as an intake/feeder, where a limit switch
   *             stops motion while intaking but should be ignored when feeding to a shooter.
   *             <p>The hardware limit faults and Forward/ReverseLimit signals will still report the
   *             values of the limit switches regardless of this parameter.
   *         <li><b>UseTimesync:</b> Set to true to delay applying this control request until a
   *             timesync boundary (requires Phoenix Pro and CANivore). This eliminates the impact
   *             of nondeterministic network delays in exchange for a larger but deterministic
   *             control latency.
   *             <p>This requires setting the ControlTimesyncFreqHz config in MotorOutputConfigs.
   *             Additionally, when this is enabled, the UpdateFreqHz of this request should be set
   *             to 0 Hz.
   *       </ul>
   * </ul>
   *
   * @param request Control object to request of the device
   * @return Code response of the request
   */
  public StatusCode setControl(DifferentialPositionDutyCycle request) {
    return talonFX.setControl(request);
  }

  /**
   * Request PID to target position with a differential position setpoint
   *
   * <p>This control mode will set the motor's position setpoint to the position specified by the
   * user. It will also set the motor's differential position setpoint to the specified position.
   *
   * <ul>
   *   <li><b>DifferentialPositionVoltage Parameters:</b>
   *       <ul>
   *         <li><b>TargetPosition:</b> Average position to drive toward in rotations.
   *         <li><b>DifferentialPosition:</b> Differential position to drive toward in rotations.
   *         <li><b>EnableFOC:</b> Set to true to use FOC commutation (requires Phoenix Pro), which
   *             increases peak power by ~15%. Set to false to use trapezoidal commutation.
   *             <p>FOC improves motor performance by leveraging torque (current) control. However,
   *             this may be inconvenient for applications that require specifying duty cycle or
   *             voltage. CTR-Electronics has developed a hybrid method that combines the
   *             performances gains of FOC while still allowing applications to provide duty cycle
   *             or voltage demand. This not to be confused with simple sinusoidal control or phase
   *             voltage control which lacks the performance gains.
   *         <li><b>TargetSlot:</b> Select which gains are applied to the primary controller by
   *             selecting the slot. Use the configuration api to set the gain values for the
   *             selected slot before enabling this feature. Slot must be within [0,2].
   *         <li><b>DifferentialSlot:</b> Select which gains are applied to the differential
   *             controller by selecting the slot. Use the configuration api to set the gain values
   *             for the selected slot before enabling this feature. Slot must be within [0,2].
   *         <li><b>OverrideBrakeDurNeutral:</b> Set to true to static-brake the rotor when output
   *             is zero (or within deadband). Set to false to use the NeutralMode configuration
   *             setting (default). This flag exists to provide the fundamental behavior of this
   *             control when output is zero, which is to provide 0V to the motor.
   *         <li><b>LimitForwardMotion:</b> Set to true to force forward limiting. This allows users
   *             to use other limit switch sensors connected to robot controller. This also allows
   *             use of active sensors that require external power.
   *         <li><b>LimitReverseMotion:</b> Set to true to force reverse limiting. This allows users
   *             to use other limit switch sensors connected to robot controller. This also allows
   *             use of active sensors that require external power.
   *         <li><b>IgnoreHardwareLimits:</b> Set to true to ignore hardware limit switches and the
   *             LimitForwardMotion and LimitReverseMotion parameters, instead allowing motion.
   *             <p>This can be useful on mechanisms such as an intake/feeder, where a limit switch
   *             stops motion while intaking but should be ignored when feeding to a shooter.
   *             <p>The hardware limit faults and Forward/ReverseLimit signals will still report the
   *             values of the limit switches regardless of this parameter.
   *         <li><b>UseTimesync:</b> Set to true to delay applying this control request until a
   *             timesync boundary (requires Phoenix Pro and CANivore). This eliminates the impact
   *             of nondeterministic network delays in exchange for a larger but deterministic
   *             control latency.
   *             <p>This requires setting the ControlTimesyncFreqHz config in MotorOutputConfigs.
   *             Additionally, when this is enabled, the UpdateFreqHz of this request should be set
   *             to 0 Hz.
   *       </ul>
   * </ul>
   *
   * @param request Control object to request of the device
   * @return Code response of the request
   */
  public StatusCode setControl(DifferentialPositionVoltage request) {
    return talonFX.setControl(request);
  }

  /**
   * Request PID to target velocity with a differential position setpoint.
   *
   * <p>This control mode will set the motor's velocity setpoint to the velocity specified by the
   * user. It will also set the motor's differential position setpoint to the specified position.
   *
   * <ul>
   *   <li><b>DifferentialVelocityDutyCycle Parameters:</b>
   *       <ul>
   *         <li><b>TargetVelocity:</b> Average velocity to drive toward in rotations per second.
   *         <li><b>DifferentialPosition:</b> Differential position to drive toward in rotations.
   *         <li><b>EnableFOC:</b> Set to true to use FOC commutation (requires Phoenix Pro), which
   *             increases peak power by ~15%. Set to false to use trapezoidal commutation.
   *             <p>FOC improves motor performance by leveraging torque (current) control. However,
   *             this may be inconvenient for applications that require specifying duty cycle or
   *             voltage. CTR-Electronics has developed a hybrid method that combines the
   *             performances gains of FOC while still allowing applications to provide duty cycle
   *             or voltage demand. This not to be confused with simple sinusoidal control or phase
   *             voltage control which lacks the performance gains.
   *         <li><b>TargetSlot:</b> Select which gains are applied to the primary controller by
   *             selecting the slot. Use the configuration api to set the gain values for the
   *             selected slot before enabling this feature. Slot must be within [0,2].
   *         <li><b>DifferentialSlot:</b> Select which gains are applied to the differential
   *             controller by selecting the slot. Use the configuration api to set the gain values
   *             for the selected slot before enabling this feature. Slot must be within [0,2].
   *         <li><b>OverrideBrakeDurNeutral:</b> Set to true to static-brake the rotor when output
   *             is zero (or within deadband). Set to false to use the NeutralMode configuration
   *             setting (default). This flag exists to provide the fundamental behavior of this
   *             control when output is zero, which is to provide 0V to the motor.
   *         <li><b>LimitForwardMotion:</b> Set to true to force forward limiting. This allows users
   *             to use other limit switch sensors connected to robot controller. This also allows
   *             use of active sensors that require external power.
   *         <li><b>LimitReverseMotion:</b> Set to true to force reverse limiting. This allows users
   *             to use other limit switch sensors connected to robot controller. This also allows
   *             use of active sensors that require external power.
   *         <li><b>IgnoreHardwareLimits:</b> Set to true to ignore hardware limit switches and the
   *             LimitForwardMotion and LimitReverseMotion parameters, instead allowing motion.
   *             <p>This can be useful on mechanisms such as an intake/feeder, where a limit switch
   *             stops motion while intaking but should be ignored when feeding to a shooter.
   *             <p>The hardware limit faults and Forward/ReverseLimit signals will still report the
   *             values of the limit switches regardless of this parameter.
   *         <li><b>UseTimesync:</b> Set to true to delay applying this control request until a
   *             timesync boundary (requires Phoenix Pro and CANivore). This eliminates the impact
   *             of nondeterministic network delays in exchange for a larger but deterministic
   *             control latency.
   *             <p>This requires setting the ControlTimesyncFreqHz config in MotorOutputConfigs.
   *             Additionally, when this is enabled, the UpdateFreqHz of this request should be set
   *             to 0 Hz.
   *       </ul>
   * </ul>
   *
   * @param request Control object to request of the device
   * @return Code response of the request
   */
  public StatusCode setControl(DifferentialVelocityDutyCycle request) {
    return talonFX.setControl(request);
  }

  /**
   * Request PID to target velocity with a differential position setpoint.
   *
   * <p>This control mode will set the motor's velocity setpoint to the velocity specified by the
   * user. It will also set the motor's differential position setpoint to the specified position.
   *
   * <ul>
   *   <li><b>DifferentialVelocityVoltage Parameters:</b>
   *       <ul>
   *         <li><b>TargetVelocity:</b> Average velocity to drive toward in rotations per second.
   *         <li><b>DifferentialPosition:</b> Differential position to drive toward in rotations.
   *         <li><b>EnableFOC:</b> Set to true to use FOC commutation (requires Phoenix Pro), which
   *             increases peak power by ~15%. Set to false to use trapezoidal commutation.
   *             <p>FOC improves motor performance by leveraging torque (current) control. However,
   *             this may be inconvenient for applications that require specifying duty cycle or
   *             voltage. CTR-Electronics has developed a hybrid method that combines the
   *             performances gains of FOC while still allowing applications to provide duty cycle
   *             or voltage demand. This not to be confused with simple sinusoidal control or phase
   *             voltage control which lacks the performance gains.
   *         <li><b>TargetSlot:</b> Select which gains are applied to the primary controller by
   *             selecting the slot. Use the configuration api to set the gain values for the
   *             selected slot before enabling this feature. Slot must be within [0,2].
   *         <li><b>DifferentialSlot:</b> Select which gains are applied to the differential
   *             controller by selecting the slot. Use the configuration api to set the gain values
   *             for the selected slot before enabling this feature. Slot must be within [0,2].
   *         <li><b>OverrideBrakeDurNeutral:</b> Set to true to static-brake the rotor when output
   *             is zero (or within deadband). Set to false to use the NeutralMode configuration
   *             setting (default). This flag exists to provide the fundamental behavior of this
   *             control when output is zero, which is to provide 0V to the motor.
   *         <li><b>LimitForwardMotion:</b> Set to true to force forward limiting. This allows users
   *             to use other limit switch sensors connected to robot controller. This also allows
   *             use of active sensors that require external power.
   *         <li><b>LimitReverseMotion:</b> Set to true to force reverse limiting. This allows users
   *             to use other limit switch sensors connected to robot controller. This also allows
   *             use of active sensors that require external power.
   *         <li><b>IgnoreHardwareLimits:</b> Set to true to ignore hardware limit switches and the
   *             LimitForwardMotion and LimitReverseMotion parameters, instead allowing motion.
   *             <p>This can be useful on mechanisms such as an intake/feeder, where a limit switch
   *             stops motion while intaking but should be ignored when feeding to a shooter.
   *             <p>The hardware limit faults and Forward/ReverseLimit signals will still report the
   *             values of the limit switches regardless of this parameter.
   *         <li><b>UseTimesync:</b> Set to true to delay applying this control request until a
   *             timesync boundary (requires Phoenix Pro and CANivore). This eliminates the impact
   *             of nondeterministic network delays in exchange for a larger but deterministic
   *             control latency.
   *             <p>This requires setting the ControlTimesyncFreqHz config in MotorOutputConfigs.
   *             Additionally, when this is enabled, the UpdateFreqHz of this request should be set
   *             to 0 Hz.
   *       </ul>
   * </ul>
   *
   * @param request Control object to request of the device
   * @return Code response of the request
   */
  public StatusCode setControl(DifferentialVelocityVoltage request) {
    return talonFX.setControl(request);
  }

  /**
   * Requests Motion Magic® to target a final position using a motion profile, and PID to a
   * differential position setpoint.
   *
   * <p>Motion Magic® produces a motion profile in real-time while attempting to honor the Cruise
   * Velocity, Acceleration, and (optional) Jerk specified via the Motion Magic® configuration
   * values. This control mode does not use the Expo_kV or Expo_kA configs.
   *
   * <p>Target position can be changed on-the-fly and Motion Magic® will do its best to adjust the
   * profile. This control mode is duty cycle based, so relevant closed-loop gains will use
   * fractional duty cycle for the numerator: +1.0 represents full forward output.
   *
   * <ul>
   *   <li><b>DifferentialMotionMagicDutyCycle Parameters:</b>
   *       <ul>
   *         <li><b>TargetPosition:</b> Average position to drive toward in rotations.
   *         <li><b>DifferentialPosition:</b> Differential position to drive toward in rotations.
   *         <li><b>EnableFOC:</b> Set to true to use FOC commutation (requires Phoenix Pro), which
   *             increases peak power by ~15%. Set to false to use trapezoidal commutation.
   *             <p>FOC improves motor performance by leveraging torque (current) control. However,
   *             this may be inconvenient for applications that require specifying duty cycle or
   *             voltage. CTR-Electronics has developed a hybrid method that combines the
   *             performances gains of FOC while still allowing applications to provide duty cycle
   *             or voltage demand. This not to be confused with simple sinusoidal control or phase
   *             voltage control which lacks the performance gains.
   *         <li><b>TargetSlot:</b> Select which gains are applied to the primary controller by
   *             selecting the slot. Use the configuration api to set the gain values for the
   *             selected slot before enabling this feature. Slot must be within [0,2].
   *         <li><b>DifferentialSlot:</b> Select which gains are applied to the differential
   *             controller by selecting the slot. Use the configuration api to set the gain values
   *             for the selected slot before enabling this feature. Slot must be within [0,2].
   *         <li><b>OverrideBrakeDurNeutral:</b> Set to true to static-brake the rotor when output
   *             is zero (or within deadband). Set to false to use the NeutralMode configuration
   *             setting (default). This flag exists to provide the fundamental behavior of this
   *             control when output is zero, which is to provide 0V to the motor.
   *         <li><b>LimitForwardMotion:</b> Set to true to force forward limiting. This allows users
   *             to use other limit switch sensors connected to robot controller. This also allows
   *             use of active sensors that require external power.
   *         <li><b>LimitReverseMotion:</b> Set to true to force reverse limiting. This allows users
   *             to use other limit switch sensors connected to robot controller. This also allows
   *             use of active sensors that require external power.
   *         <li><b>IgnoreHardwareLimits:</b> Set to true to ignore hardware limit switches and the
   *             LimitForwardMotion and LimitReverseMotion parameters, instead allowing motion.
   *             <p>This can be useful on mechanisms such as an intake/feeder, where a limit switch
   *             stops motion while intaking but should be ignored when feeding to a shooter.
   *             <p>The hardware limit faults and Forward/ReverseLimit signals will still report the
   *             values of the limit switches regardless of this parameter.
   *         <li><b>UseTimesync:</b> Set to true to delay applying this control request until a
   *             timesync boundary (requires Phoenix Pro and CANivore). This eliminates the impact
   *             of nondeterministic network delays in exchange for a larger but deterministic
   *             control latency.
   *             <p>This requires setting the ControlTimesyncFreqHz config in MotorOutputConfigs.
   *             Additionally, when this is enabled, the UpdateFreqHz of this request should be set
   *             to 0 Hz.
   *       </ul>
   * </ul>
   *
   * @param request Control object to request of the device
   * @return Code response of the request
   */
  public StatusCode setControl(DifferentialMotionMagicDutyCycle request) {
    return talonFX.setControl(request);
  }

  /**
   * Requests Motion Magic® to target a final position using a motion profile, and PID to a
   * differential position setpoint.
   *
   * <p>Motion Magic® produces a motion profile in real-time while attempting to honor the Cruise
   * Velocity, Acceleration, and (optional) Jerk specified via the Motion Magic® configuration
   * values. This control mode does not use the Expo_kV or Expo_kA configs.
   *
   * <p>Target position can be changed on-the-fly and Motion Magic® will do its best to adjust the
   * profile. This control mode is voltage-based, so relevant closed-loop gains will use Volts for
   * the numerator.
   *
   * <ul>
   *   <li><b>DifferentialMotionMagicVoltage Parameters:</b>
   *       <ul>
   *         <li><b>TargetPosition:</b> Average position to drive toward in rotations.
   *         <li><b>DifferentialPosition:</b> Differential position to drive toward in rotations.
   *         <li><b>EnableFOC:</b> Set to true to use FOC commutation (requires Phoenix Pro), which
   *             increases peak power by ~15%. Set to false to use trapezoidal commutation.
   *             <p>FOC improves motor performance by leveraging torque (current) control. However,
   *             this may be inconvenient for applications that require specifying duty cycle or
   *             voltage. CTR-Electronics has developed a hybrid method that combines the
   *             performances gains of FOC while still allowing applications to provide duty cycle
   *             or voltage demand. This not to be confused with simple sinusoidal control or phase
   *             voltage control which lacks the performance gains.
   *         <li><b>TargetSlot:</b> Select which gains are applied to the primary controller by
   *             selecting the slot. Use the configuration api to set the gain values for the
   *             selected slot before enabling this feature. Slot must be within [0,2].
   *         <li><b>DifferentialSlot:</b> Select which gains are applied to the differential
   *             controller by selecting the slot. Use the configuration api to set the gain values
   *             for the selected slot before enabling this feature. Slot must be within [0,2].
   *         <li><b>OverrideBrakeDurNeutral:</b> Set to true to static-brake the rotor when output
   *             is zero (or within deadband). Set to false to use the NeutralMode configuration
   *             setting (default). This flag exists to provide the fundamental behavior of this
   *             control when output is zero, which is to provide 0V to the motor.
   *         <li><b>LimitForwardMotion:</b> Set to true to force forward limiting. This allows users
   *             to use other limit switch sensors connected to robot controller. This also allows
   *             use of active sensors that require external power.
   *         <li><b>LimitReverseMotion:</b> Set to true to force reverse limiting. This allows users
   *             to use other limit switch sensors connected to robot controller. This also allows
   *             use of active sensors that require external power.
   *         <li><b>IgnoreHardwareLimits:</b> Set to true to ignore hardware limit switches and the
   *             LimitForwardMotion and LimitReverseMotion parameters, instead allowing motion.
   *             <p>This can be useful on mechanisms such as an intake/feeder, where a limit switch
   *             stops motion while intaking but should be ignored when feeding to a shooter.
   *             <p>The hardware limit faults and Forward/ReverseLimit signals will still report the
   *             values of the limit switches regardless of this parameter.
   *         <li><b>UseTimesync:</b> Set to true to delay applying this control request until a
   *             timesync boundary (requires Phoenix Pro and CANivore). This eliminates the impact
   *             of nondeterministic network delays in exchange for a larger but deterministic
   *             control latency.
   *             <p>This requires setting the ControlTimesyncFreqHz config in MotorOutputConfigs.
   *             Additionally, when this is enabled, the UpdateFreqHz of this request should be set
   *             to 0 Hz.
   *       </ul>
   * </ul>
   *
   * @param request Control object to request of the device
   * @return Code response of the request
   */
  public StatusCode setControl(DifferentialMotionMagicVoltage request) {
    return talonFX.setControl(request);
  }

  /**
   * Follow the motor output of another Talon.
   *
   * <p>If Talon is in torque control, the torque is copied - which will increase the total torque
   * applied. If Talon is in percent supply output control, the duty cycle is matched. Motor
   * direction either matches master's configured direction or opposes it based on
   * OpposeMasterDirection.
   *
   * <ul>
   *   <li><b>Follower Parameters:</b>
   *       <ul>
   *         <li><b>MasterID:</b> Device ID of the master to follow.
   *         <li><b>OpposeMasterDirection:</b> Set to false for motor invert to match the master's
   *             configured Invert - which is typical when master and follower are mechanically
   *             linked and spin in the same direction. Set to true for motor invert to oppose the
   *             master's configured Invert - this is typical where the the master and follower
   *             mechanically spin in opposite directions.
   *       </ul>
   * </ul>
   *
   * @param request Control object to request of the device
   * @return Code response of the request
   */
  public StatusCode setControl(Follower request) {
    return talonFX.setControl(request);
  }

  /**
   * Follow the motor output of another Talon while ignoring the master's invert setting.
   *
   * <p>If Talon is in torque control, the torque is copied - which will increase the total torque
   * applied. If Talon is in percent supply output control, the duty cycle is matched. Motor
   * direction is strictly determined by the configured invert and not the master. If you want motor
   * direction to match or oppose the master, use FollowerRequest instead.
   *
   * <ul>
   *   <li><b>StrictFollower Parameters:</b>
   *       <ul>
   *         <li><b>MasterID:</b> Device ID of the master to follow.
   *       </ul>
   * </ul>
   *
   * @param request Control object to request of the device
   * @return Code response of the request
   */
  public StatusCode setControl(StrictFollower request) {
    return talonFX.setControl(request);
  }

  /**
   * Follow the differential motor output of another Talon.
   *
   * <p>If Talon is in torque control, the torque is copied - which will increase the total torque
   * applied. If Talon is in percent supply output control, the duty cycle is matched. Motor
   * direction either matches master's configured direction or opposes it based on
   * OpposeMasterDirection.
   *
   * <ul>
   *   <li><b>DifferentialFollower Parameters:</b>
   *       <ul>
   *         <li><b>MasterID:</b> Device ID of the differential master to follow.
   *         <li><b>OpposeMasterDirection:</b> Set to false for motor invert to match the master's
   *             configured Invert - which is typical when master and follower are mechanically
   *             linked and spin in the same direction. Set to true for motor invert to oppose the
   *             master's configured Invert - this is typical where the the master and follower
   *             mechanically spin in opposite directions.
   *       </ul>
   * </ul>
   *
   * @param request Control object to request of the device
   * @return Code response of the request
   */
  public StatusCode setControl(DifferentialFollower request) {
    return talonFX.setControl(request);
  }

  /**
   * Follow the differential motor output of another Talon while ignoring the master's invert
   * setting.
   *
   * <p>If Talon is in torque control, the torque is copied - which will increase the total torque
   * applied. If Talon is in percent supply output control, the duty cycle is matched. Motor
   * direction is strictly determined by the configured invert and not the master. If you want motor
   * direction to match or oppose the master, use FollowerRequest instead.
   *
   * <ul>
   *   <li><b>DifferentialStrictFollower Parameters:</b>
   *       <ul>
   *         <li><b>MasterID:</b> Device ID of the differential master to follow.
   *       </ul>
   * </ul>
   *
   * @param request Control object to request of the device
   * @return Code response of the request
   */
  public StatusCode setControl(DifferentialStrictFollower request) {
    return talonFX.setControl(request);
  }

  /**
   * Request neutral output of actuator. The applied brake type is determined by the NeutralMode
   * configuration.
   *
   * <ul>
   *   <li><b>NeutralOut Parameters:</b>
   *       <ul>
   *         <li><b>UseTimesync:</b> Set to true to delay applying this control request until a
   *             timesync boundary (requires Phoenix Pro and CANivore). This eliminates the impact
   *             of nondeterministic network delays in exchange for a larger but deterministic
   *             control latency.
   *             <p>This requires setting the ControlTimesyncFreqHz config in MotorOutputConfigs.
   *             Additionally, when this is enabled, the UpdateFreqHz of this request should be set
   *             to 0 Hz.
   *       </ul>
   * </ul>
   *
   * @param request Control object to request of the device
   * @return Code response of the request
   */
  public StatusCode setControl(NeutralOut request) {
    return talonFX.setControl(request);
  }

  /**
   * Request coast neutral output of actuator. The bridge is disabled and the rotor is allowed to
   * coast.
   *
   * <ul>
   *   <li><b>CoastOut Parameters:</b>
   *       <ul>
   *         <li><b>UseTimesync:</b> Set to true to delay applying this control request until a
   *             timesync boundary (requires Phoenix Pro and CANivore). This eliminates the impact
   *             of nondeterministic network delays in exchange for a larger but deterministic
   *             control latency.
   *             <p>This requires setting the ControlTimesyncFreqHz config in MotorOutputConfigs.
   *             Additionally, when this is enabled, the UpdateFreqHz of this request should be set
   *             to 0 Hz.
   *       </ul>
   * </ul>
   *
   * @param request Control object to request of the device
   * @return Code response of the request
   */
  public StatusCode setControl(CoastOut request) {
    return talonFX.setControl(request);
  }

  /**
   * Applies full neutral-brake by shorting motor leads together.
   *
   * <ul>
   *   <li><b>StaticBrake Parameters:</b>
   *       <ul>
   *         <li><b>UseTimesync:</b> Set to true to delay applying this control request until a
   *             timesync boundary (requires Phoenix Pro and CANivore). This eliminates the impact
   *             of nondeterministic network delays in exchange for a larger but deterministic
   *             control latency.
   *             <p>This requires setting the ControlTimesyncFreqHz config in MotorOutputConfigs.
   *             Additionally, when this is enabled, the UpdateFreqHz of this request should be set
   *             to 0 Hz.
   *       </ul>
   * </ul>
   *
   * @param request Control object to request of the device
   * @return Code response of the request
   */
  public StatusCode setControl(StaticBrake request) {
    return talonFX.setControl(request);
  }

  /**
   * Plays a single tone at the user specified frequency.
   *
   * <ul>
   *   <li><b>MusicTone Parameters:</b>
   *       <ul>
   *         <li><b>AudioFrequency:</b> Sound frequency to play. A value of zero will silence the
   *             device. The effective frequency range is 10-20000 Hz. Any nonzero frequency less
   *             than 10 Hz will be capped to 10 Hz. Any frequency above 20 kHz will be capped to 20
   *             kHz.
   *       </ul>
   * </ul>
   *
   * @param request Control object to request of the device
   * @return Code response of the request
   */
  public StatusCode setControl(MusicTone request) {
    return talonFX.setControl(request);
  }

  /**
   * Requests Motion Magic® to target a final velocity using a motion profile. This allows smooth
   * transitions between velocity set points. Users can optionally provide a duty cycle feedforward.
   *
   * <p>Motion Magic® Velocity produces a motion profile in real-time while attempting to honor the
   * specified Acceleration and (optional) Jerk. This control mode does not use the CruiseVelocity,
   * Expo_kV, or Expo_kA configs.
   *
   * <p>If the specified acceleration is zero, the Acceleration under Motion Magic® configuration
   * parameter is used instead. This allows for runtime adjustment of acceleration for advanced
   * users. Jerk is also specified in the Motion Magic® persistent configuration values. If Jerk is
   * set to zero, Motion Magic® will produce a trapezoidal acceleration profile.
   *
   * <p>Target velocity can also be changed on-the-fly and Motion Magic® will do its best to adjust
   * the profile. This control mode is duty cycle based, so relevant closed-loop gains will use
   * fractional duty cycle for the numerator: +1.0 represents full forward output.
   *
   * <ul>
   *   <li><b>MotionMagicVelocityDutyCycle Parameters:</b>
   *       <ul>
   *         <li><b>Velocity:</b> Target velocity to drive toward in rotations per second. This can
   *             be changed on-the fly.
   *         <li><b>Acceleration:</b> This is the absolute Acceleration to use generating the
   *             profile. If this parameter is zero, the Acceleration persistent configuration
   *             parameter is used instead. Acceleration is in rotations per second squared. If
   *             nonzero, the signage does not matter as the absolute value is used.
   *         <li><b>EnableFOC:</b> Set to true to use FOC commutation (requires Phoenix Pro), which
   *             increases peak power by ~15%. Set to false to use trapezoidal commutation.
   *             <p>FOC improves motor performance by leveraging torque (current) control. However,
   *             this may be inconvenient for applications that require specifying duty cycle or
   *             voltage. CTR-Electronics has developed a hybrid method that combines the
   *             performances gains of FOC while still allowing applications to provide duty cycle
   *             or voltage demand. This not to be confused with simple sinusoidal control or phase
   *             voltage control which lacks the performance gains.
   *         <li><b>FeedForward:</b> Feedforward to apply in fractional units between -1 and +1.
   *         <li><b>Slot:</b> Select which gains are applied by selecting the slot. Use the
   *             configuration api to set the gain values for the selected slot before enabling this
   *             feature. Slot must be within [0,2].
   *         <li><b>OverrideBrakeDurNeutral:</b> Set to true to static-brake the rotor when output
   *             is zero (or within deadband). Set to false to use the NeutralMode configuration
   *             setting (default). This flag exists to provide the fundamental behavior of this
   *             control when output is zero, which is to provide 0V to the motor.
   *         <li><b>LimitForwardMotion:</b> Set to true to force forward limiting. This allows users
   *             to use other limit switch sensors connected to robot controller. This also allows
   *             use of active sensors that require external power.
   *         <li><b>LimitReverseMotion:</b> Set to true to force reverse limiting. This allows users
   *             to use other limit switch sensors connected to robot controller. This also allows
   *             use of active sensors that require external power.
   *         <li><b>IgnoreHardwareLimits:</b> Set to true to ignore hardware limit switches and the
   *             LimitForwardMotion and LimitReverseMotion parameters, instead allowing motion.
   *             <p>This can be useful on mechanisms such as an intake/feeder, where a limit switch
   *             stops motion while intaking but should be ignored when feeding to a shooter.
   *             <p>The hardware limit faults and Forward/ReverseLimit signals will still report the
   *             values of the limit switches regardless of this parameter.
   *         <li><b>UseTimesync:</b> Set to true to delay applying this control request until a
   *             timesync boundary (requires Phoenix Pro and CANivore). This eliminates the impact
   *             of nondeterministic network delays in exchange for a larger but deterministic
   *             control latency.
   *             <p>This requires setting the ControlTimesyncFreqHz config in MotorOutputConfigs.
   *             Additionally, when this is enabled, the UpdateFreqHz of this request should be set
   *             to 0 Hz.
   *       </ul>
   * </ul>
   *
   * @param request Control object to request of the device
   * @return Code response of the request
   */
  public StatusCode setControl(MotionMagicVelocityDutyCycle request) {
    return talonFX.setControl(request);
  }

  /**
   * Requests Motion Magic® to target a final velocity using a motion profile. This allows smooth
   * transitions between velocity set points. Users can optionally provide a torque feedforward.
   *
   * <p>Motion Magic® Velocity produces a motion profile in real-time while attempting to honor the
   * specified Acceleration and (optional) Jerk. This control mode does not use the CruiseVelocity,
   * Expo_kV, or Expo_kA configs.
   *
   * <p>If the specified acceleration is zero, the Acceleration under Motion Magic® configuration
   * parameter is used instead. This allows for runtime adjustment of acceleration for advanced
   * users. Jerk is also specified in the Motion Magic® persistent configuration values. If Jerk is
   * set to zero, Motion Magic® will produce a trapezoidal acceleration profile.
   *
   * <p>Target velocity can also be changed on-the-fly and Motion Magic® will do its best to adjust
   * the profile. This control mode is based on torque current, so relevant closed-loop gains will
   * use Amperes for the numerator.
   *
   * <ul>
   *   <li><b>MotionMagicVelocityTorqueCurrentFOC Parameters:</b>
   *       <ul>
   *         <li><b>Velocity:</b> Target velocity to drive toward in rotations per second. This can
   *             be changed on-the fly.
   *         <li><b>Acceleration:</b> This is the absolute Acceleration to use generating the
   *             profile. If this parameter is zero, the Acceleration persistent configuration
   *             parameter is used instead. Acceleration is in rotations per second squared. If
   *             nonzero, the signage does not matter as the absolute value is used.
   *         <li><b>EnableFOC:</b> Set to true to use FOC commutation (requires Phoenix Pro), which
   *             increases peak power by ~15%. Set to false to use trapezoidal commutation.
   *             <p>FOC improves motor performance by leveraging torque (current) control. However,
   *             this may be inconvenient for applications that require specifying duty cycle or
   *             voltage. CTR-Electronics has developed a hybrid method that combines the
   *             performances gains of FOC while still allowing applications to provide duty cycle
   *             or voltage demand. This not to be confused with simple sinusoidal control or phase
   *             voltage control which lacks the performance gains.
   *         <li><b>FeedForward:</b> Feedforward to apply in torque current in Amperes. User can use
   *             motor's kT to scale Newton-meter to Amperes.
   *         <li><b>Slot:</b> Select which gains are applied by selecting the slot. Use the
   *             configuration api to set the gain values for the selected slot before enabling this
   *             feature. Slot must be within [0,2].
   *         <li><b>OverrideCoastDurNeutral:</b> Set to true to coast the rotor when output is zero
   *             (or within deadband). Set to false to use the NeutralMode configuration setting
   *             (default). This flag exists to provide the fundamental behavior of this control
   *             when output is zero, which is to provide 0A (zero torque).
   *         <li><b>LimitForwardMotion:</b> Set to true to force forward limiting. This allows users
   *             to use other limit switch sensors connected to robot controller. This also allows
   *             use of active sensors that require external power.
   *         <li><b>LimitReverseMotion:</b> Set to true to force reverse limiting. This allows users
   *             to use other limit switch sensors connected to robot controller. This also allows
   *             use of active sensors that require external power.
   *         <li><b>IgnoreHardwareLimits:</b> Set to true to ignore hardware limit switches and the
   *             LimitForwardMotion and LimitReverseMotion parameters, instead allowing motion.
   *             <p>This can be useful on mechanisms such as an intake/feeder, where a limit switch
   *             stops motion while intaking but should be ignored when feeding to a shooter.
   *             <p>The hardware limit faults and Forward/ReverseLimit signals will still report the
   *             values of the limit switches regardless of this parameter.
   *         <li><b>UseTimesync:</b> Set to true to delay applying this control request until a
   *             timesync boundary (requires Phoenix Pro and CANivore). This eliminates the impact
   *             of nondeterministic network delays in exchange for a larger but deterministic
   *             control latency.
   *             <p>This requires setting the ControlTimesyncFreqHz config in MotorOutputConfigs.
   *             Additionally, when this is enabled, the UpdateFreqHz of this request should be set
   *             to 0 Hz.
   *       </ul>
   * </ul>
   *
   * @param request Control object to request of the device
   * @return Code response of the request
   */
  public StatusCode setControl(MotionMagicVelocityTorqueCurrentFOC request) {
    return talonFX.setControl(request);
  }

  /**
   * Requests Motion Magic® to target a final velocity using a motion profile. This allows smooth
   * transitions between velocity set points. Users can optionally provide a voltage feedforward.
   *
   * <p>Motion Magic® Velocity produces a motion profile in real-time while attempting to honor the
   * specified Acceleration and (optional) Jerk. This control mode does not use the CruiseVelocity,
   * Expo_kV, or Expo_kA configs.
   *
   * <p>If the specified acceleration is zero, the Acceleration under Motion Magic® configuration
   * parameter is used instead. This allows for runtime adjustment of acceleration for advanced
   * users. Jerk is also specified in the Motion Magic® persistent configuration values. If Jerk is
   * set to zero, Motion Magic® will produce a trapezoidal acceleration profile.
   *
   * <p>Target velocity can also be changed on-the-fly and Motion Magic® will do its best to adjust
   * the profile. This control mode is voltage-based, so relevant closed-loop gains will use Volts
   * for the numerator.
   *
   * <ul>
   *   <li><b>MotionMagicVelocityVoltage Parameters:</b>
   *       <ul>
   *         <li><b>Velocity:</b> Target velocity to drive toward in rotations per second. This can
   *             be changed on-the fly.
   *         <li><b>Acceleration:</b> This is the absolute Acceleration to use generating the
   *             profile. If this parameter is zero, the Acceleration persistent configuration
   *             parameter is used instead. Acceleration is in rotations per second squared. If
   *             nonzero, the signage does not matter as the absolute value is used.
   *         <li><b>EnableFOC:</b> Set to true to use FOC commutation (requires Phoenix Pro), which
   *             increases peak power by ~15%. Set to false to use trapezoidal commutation.
   *             <p>FOC improves motor performance by leveraging torque (current) control. However,
   *             this may be inconvenient for applications that require specifying duty cycle or
   *             voltage. CTR-Electronics has developed a hybrid method that combines the
   *             performances gains of FOC while still allowing applications to provide duty cycle
   *             or voltage demand. This not to be confused with simple sinusoidal control or phase
   *             voltage control which lacks the performance gains.
   *         <li><b>FeedForward:</b> Feedforward to apply in volts
   *         <li><b>Slot:</b> Select which gains are applied by selecting the slot. Use the
   *             configuration api to set the gain values for the selected slot before enabling this
   *             feature. Slot must be within [0,2].
   *         <li><b>OverrideBrakeDurNeutral:</b> Set to true to static-brake the rotor when output
   *             is zero (or within deadband). Set to false to use the NeutralMode configuration
   *             setting (default). This flag exists to provide the fundamental behavior of this
   *             control when output is zero, which is to provide 0V to the motor.
   *         <li><b>LimitForwardMotion:</b> Set to true to force forward limiting. This allows users
   *             to use other limit switch sensors connected to robot controller. This also allows
   *             use of active sensors that require external power.
   *         <li><b>LimitReverseMotion:</b> Set to true to force reverse limiting. This allows users
   *             to use other limit switch sensors connected to robot controller. This also allows
   *             use of active sensors that require external power.
   *         <li><b>IgnoreHardwareLimits:</b> Set to true to ignore hardware limit switches and the
   *             LimitForwardMotion and LimitReverseMotion parameters, instead allowing motion.
   *             <p>This can be useful on mechanisms such as an intake/feeder, where a limit switch
   *             stops motion while intaking but should be ignored when feeding to a shooter.
   *             <p>The hardware limit faults and Forward/ReverseLimit signals will still report the
   *             values of the limit switches regardless of this parameter.
   *         <li><b>UseTimesync:</b> Set to true to delay applying this control request until a
   *             timesync boundary (requires Phoenix Pro and CANivore). This eliminates the impact
   *             of nondeterministic network delays in exchange for a larger but deterministic
   *             control latency.
   *             <p>This requires setting the ControlTimesyncFreqHz config in MotorOutputConfigs.
   *             Additionally, when this is enabled, the UpdateFreqHz of this request should be set
   *             to 0 Hz.
   *       </ul>
   * </ul>
   *
   * @param request Control object to request of the device
   * @return Code response of the request
   */
  public StatusCode setControl(MotionMagicVelocityVoltage request) {
    return talonFX.setControl(request);
  }

  /**
   * Requests Motion Magic® to target a final position using an exponential motion profile. Users
   * can optionally provide a duty cycle feedforward.
   *
   * <p>Motion Magic® Expo produces a motion profile in real-time while attempting to honor the
   * Cruise Velocity (optional) and the mechanism kV and kA, specified via the Motion Magic®
   * configuration values. Note that unlike the slot gains, the Expo_kV and Expo_kA configs are
   * always in output units of Volts.
   *
   * <p>Setting Cruise Velocity to 0 will allow the profile to run to the max possible velocity
   * based on Expo_kV. This control mode does not use the Acceleration or Jerk configs.
   *
   * <p>Target position can be changed on-the-fly and Motion Magic® will do its best to adjust the
   * profile. This control mode is duty cycle based, so relevant closed-loop gains will use
   * fractional duty cycle for the numerator: +1.0 represents full forward output.
   *
   * <ul>
   *   <li><b>MotionMagicExpoDutyCycle Parameters:</b>
   *       <ul>
   *         <li><b>Position:</b> Position to drive toward in rotations.
   *         <li><b>EnableFOC:</b> Set to true to use FOC commutation (requires Phoenix Pro), which
   *             increases peak power by ~15%. Set to false to use trapezoidal commutation.
   *             <p>FOC improves motor performance by leveraging torque (current) control. However,
   *             this may be inconvenient for applications that require specifying duty cycle or
   *             voltage. CTR-Electronics has developed a hybrid method that combines the
   *             performances gains of FOC while still allowing applications to provide duty cycle
   *             or voltage demand. This not to be confused with simple sinusoidal control or phase
   *             voltage control which lacks the performance gains.
   *         <li><b>FeedForward:</b> Feedforward to apply in fractional units between -1 and +1.
   *         <li><b>Slot:</b> Select which gains are applied by selecting the slot. Use the
   *             configuration api to set the gain values for the selected slot before enabling this
   *             feature. Slot must be within [0,2].
   *         <li><b>OverrideBrakeDurNeutral:</b> Set to true to static-brake the rotor when output
   *             is zero (or within deadband). Set to false to use the NeutralMode configuration
   *             setting (default). This flag exists to provide the fundamental behavior of this
   *             control when output is zero, which is to provide 0V to the motor.
   *         <li><b>LimitForwardMotion:</b> Set to true to force forward limiting. This allows users
   *             to use other limit switch sensors connected to robot controller. This also allows
   *             use of active sensors that require external power.
   *         <li><b>LimitReverseMotion:</b> Set to true to force reverse limiting. This allows users
   *             to use other limit switch sensors connected to robot controller. This also allows
   *             use of active sensors that require external power.
   *         <li><b>IgnoreHardwareLimits:</b> Set to true to ignore hardware limit switches and the
   *             LimitForwardMotion and LimitReverseMotion parameters, instead allowing motion.
   *             <p>This can be useful on mechanisms such as an intake/feeder, where a limit switch
   *             stops motion while intaking but should be ignored when feeding to a shooter.
   *             <p>The hardware limit faults and Forward/ReverseLimit signals will still report the
   *             values of the limit switches regardless of this parameter.
   *         <li><b>UseTimesync:</b> Set to true to delay applying this control request until a
   *             timesync boundary (requires Phoenix Pro and CANivore). This eliminates the impact
   *             of nondeterministic network delays in exchange for a larger but deterministic
   *             control latency.
   *             <p>This requires setting the ControlTimesyncFreqHz config in MotorOutputConfigs.
   *             Additionally, when this is enabled, the UpdateFreqHz of this request should be set
   *             to 0 Hz.
   *       </ul>
   * </ul>
   *
   * @param request Control object to request of the device
   * @return Code response of the request
   */
  public StatusCode setControl(MotionMagicExpoDutyCycle request) {
    return talonFX.setControl(request);
  }

  /**
   * Requests Motion Magic® to target a final position using an exponential motion profile. Users
   * can optionally provide a voltage feedforward.
   *
   * <p>Motion Magic® Expo produces a motion profile in real-time while attempting to honor the
   * Cruise Velocity (optional) and the mechanism kV and kA, specified via the Motion Magic®
   * configuration values. Note that unlike the slot gains, the Expo_kV and Expo_kA configs are
   * always in output units of Volts.
   *
   * <p>Setting Cruise Velocity to 0 will allow the profile to run to the max possible velocity
   * based on Expo_kV. This control mode does not use the Acceleration or Jerk configs.
   *
   * <p>Target position can be changed on-the-fly and Motion Magic® will do its best to adjust the
   * profile. This control mode is voltage-based, so relevant closed-loop gains will use Volts for
   * the numerator.
   *
   * <ul>
   *   <li><b>MotionMagicExpoVoltage Parameters:</b>
   *       <ul>
   *         <li><b>Position:</b> Position to drive toward in rotations.
   *         <li><b>EnableFOC:</b> Set to true to use FOC commutation (requires Phoenix Pro), which
   *             increases peak power by ~15%. Set to false to use trapezoidal commutation.
   *             <p>FOC improves motor performance by leveraging torque (current) control. However,
   *             this may be inconvenient for applications that require specifying duty cycle or
   *             voltage. CTR-Electronics has developed a hybrid method that combines the
   *             performances gains of FOC while still allowing applications to provide duty cycle
   *             or voltage demand. This not to be confused with simple sinusoidal control or phase
   *             voltage control which lacks the performance gains.
   *         <li><b>FeedForward:</b> Feedforward to apply in volts
   *         <li><b>Slot:</b> Select which gains are applied by selecting the slot. Use the
   *             configuration api to set the gain values for the selected slot before enabling this
   *             feature. Slot must be within [0,2].
   *         <li><b>OverrideBrakeDurNeutral:</b> Set to true to static-brake the rotor when output
   *             is zero (or within deadband). Set to false to use the NeutralMode configuration
   *             setting (default). This flag exists to provide the fundamental behavior of this
   *             control when output is zero, which is to provide 0V to the motor.
   *         <li><b>LimitForwardMotion:</b> Set to true to force forward limiting. This allows users
   *             to use other limit switch sensors connected to robot controller. This also allows
   *             use of active sensors that require external power.
   *         <li><b>LimitReverseMotion:</b> Set to true to force reverse limiting. This allows users
   *             to use other limit switch sensors connected to robot controller. This also allows
   *             use of active sensors that require external power.
   *         <li><b>IgnoreHardwareLimits:</b> Set to true to ignore hardware limit switches and the
   *             LimitForwardMotion and LimitReverseMotion parameters, instead allowing motion.
   *             <p>This can be useful on mechanisms such as an intake/feeder, where a limit switch
   *             stops motion while intaking but should be ignored when feeding to a shooter.
   *             <p>The hardware limit faults and Forward/ReverseLimit signals will still report the
   *             values of the limit switches regardless of this parameter.
   *         <li><b>UseTimesync:</b> Set to true to delay applying this control request until a
   *             timesync boundary (requires Phoenix Pro and CANivore). This eliminates the impact
   *             of nondeterministic network delays in exchange for a larger but deterministic
   *             control latency.
   *             <p>This requires setting the ControlTimesyncFreqHz config in MotorOutputConfigs.
   *             Additionally, when this is enabled, the UpdateFreqHz of this request should be set
   *             to 0 Hz.
   *       </ul>
   * </ul>
   *
   * @param request Control object to request of the device
   * @return Code response of the request
   */
  public StatusCode setControl(MotionMagicExpoVoltage request) {
    return talonFX.setControl(request);
  }

  /**
   * Requests Motion Magic® to target a final position using an exponential motion profile. Users
   * can optionally provide a torque current feedforward.
   *
   * <p>Motion Magic® Expo produces a motion profile in real-time while attempting to honor the
   * Cruise Velocity (optional) and the mechanism kV and kA, specified via the Motion Magic®
   * configuration values. Note that unlike the slot gains, the Expo_kV and Expo_kA configs are
   * always in output units of Volts.
   *
   * <p>Setting Cruise Velocity to 0 will allow the profile to run to the max possible velocity
   * based on Expo_kV. This control mode does not use the Acceleration or Jerk configs.
   *
   * <p>Target position can be changed on-the-fly and Motion Magic® will do its best to adjust the
   * profile. This control mode is based on torque current, so relevant closed-loop gains will use
   * Amperes for the numerator.
   *
   * <ul>
   *   <li><b>MotionMagicExpoTorqueCurrentFOC Parameters:</b>
   *       <ul>
   *         <li><b>Position:</b> Position to drive toward in rotations.
   *         <li><b>FeedForward:</b> Feedforward to apply in torque current in Amperes. User can use
   *             motor's kT to scale Newton-meter to Amperes.
   *         <li><b>Slot:</b> Select which gains are applied by selecting the slot. Use the
   *             configuration api to set the gain values for the selected slot before enabling this
   *             feature. Slot must be within [0,2].
   *         <li><b>OverrideCoastDurNeutral:</b> Set to true to coast the rotor when output is zero
   *             (or within deadband). Set to false to use the NeutralMode configuration setting
   *             (default). This flag exists to provide the fundamental behavior of this control
   *             when output is zero, which is to provide 0A (zero torque).
   *         <li><b>LimitForwardMotion:</b> Set to true to force forward limiting. This allows users
   *             to use other limit switch sensors connected to robot controller. This also allows
   *             use of active sensors that require external power.
   *         <li><b>LimitReverseMotion:</b> Set to true to force reverse limiting. This allows users
   *             to use other limit switch sensors connected to robot controller. This also allows
   *             use of active sensors that require external power.
   *         <li><b>IgnoreHardwareLimits:</b> Set to true to ignore hardware limit switches and the
   *             LimitForwardMotion and LimitReverseMotion parameters, instead allowing motion.
   *             <p>This can be useful on mechanisms such as an intake/feeder, where a limit switch
   *             stops motion while intaking but should be ignored when feeding to a shooter.
   *             <p>The hardware limit faults and Forward/ReverseLimit signals will still report the
   *             values of the limit switches regardless of this parameter.
   *         <li><b>UseTimesync:</b> Set to true to delay applying this control request until a
   *             timesync boundary (requires Phoenix Pro and CANivore). This eliminates the impact
   *             of nondeterministic network delays in exchange for a larger but deterministic
   *             control latency.
   *             <p>This requires setting the ControlTimesyncFreqHz config in MotorOutputConfigs.
   *             Additionally, when this is enabled, the UpdateFreqHz of this request should be set
   *             to 0 Hz.
   *       </ul>
   * </ul>
   *
   * @param request Control object to request of the device
   * @return Code response of the request
   */
  public StatusCode setControl(MotionMagicExpoTorqueCurrentFOC request) {
    return talonFX.setControl(request);
  }

  /**
   * Requests Motion Magic® to target a final position using a motion profile. This dynamic request
   * allows runtime changes to Cruise Velocity, Acceleration, and Jerk. Users can optionally provide
   * a duty cycle feedforward. This control requires use of a CANivore.
   *
   * <p>Motion Magic® produces a motion profile in real-time while attempting to honor the specified
   * Cruise Velocity, Acceleration, and (optional) Jerk. This control mode does not use the Expo_kV
   * or Expo_kA configs.
   *
   * <p>Target position can be changed on-the-fly and Motion Magic® will do its best to adjust the
   * profile. This control mode is duty cycle based, so relevant closed-loop gains will use
   * fractional duty cycle for the numerator: +1.0 represents full forward output.
   *
   * <ul>
   *   <li><b>DynamicMotionMagicDutyCycle Parameters:</b>
   *       <ul>
   *         <li><b>Position:</b> Position to drive toward in rotations.
   *         <li><b>Velocity:</b> Cruise velocity for profiling. The signage does not matter as the
   *             device will use the absolute value for profile generation.
   *         <li><b>Acceleration:</b> Acceleration for profiling. The signage does not matter as the
   *             device will use the absolute value for profile generation
   *         <li><b>Jerk:</b> Jerk for profiling. The signage does not matter as the device will use
   *             the absolute value for profile generation.
   *             <p>Jerk is optional; if this is set to zero, then Motion Magic® will not apply a
   *             Jerk limit.
   *         <li><b>EnableFOC:</b> Set to true to use FOC commutation (requires Phoenix Pro), which
   *             increases peak power by ~15%. Set to false to use trapezoidal commutation.
   *             <p>FOC improves motor performance by leveraging torque (current) control. However,
   *             this may be inconvenient for applications that require specifying duty cycle or
   *             voltage. CTR-Electronics has developed a hybrid method that combines the
   *             performances gains of FOC while still allowing applications to provide duty cycle
   *             or voltage demand. This not to be confused with simple sinusoidal control or phase
   *             voltage control which lacks the performance gains.
   *         <li><b>FeedForward:</b> Feedforward to apply in fractional units between -1 and +1.
   *         <li><b>Slot:</b> Select which gains are applied by selecting the slot. Use the
   *             configuration api to set the gain values for the selected slot before enabling this
   *             feature. Slot must be within [0,2].
   *         <li><b>OverrideBrakeDurNeutral:</b> Set to true to static-brake the rotor when output
   *             is zero (or within deadband). Set to false to use the NeutralMode configuration
   *             setting (default). This flag exists to provide the fundamental behavior of this
   *             control when output is zero, which is to provide 0V to the motor.
   *         <li><b>LimitForwardMotion:</b> Set to true to force forward limiting. This allows users
   *             to use other limit switch sensors connected to robot controller. This also allows
   *             use of active sensors that require external power.
   *         <li><b>LimitReverseMotion:</b> Set to true to force reverse limiting. This allows users
   *             to use other limit switch sensors connected to robot controller. This also allows
   *             use of active sensors that require external power.
   *         <li><b>IgnoreHardwareLimits:</b> Set to true to ignore hardware limit switches and the
   *             LimitForwardMotion and LimitReverseMotion parameters, instead allowing motion.
   *             <p>This can be useful on mechanisms such as an intake/feeder, where a limit switch
   *             stops motion while intaking but should be ignored when feeding to a shooter.
   *             <p>The hardware limit faults and Forward/ReverseLimit signals will still report the
   *             values of the limit switches regardless of this parameter.
   *         <li><b>UseTimesync:</b> Set to true to delay applying this control request until a
   *             timesync boundary (requires Phoenix Pro and CANivore). This eliminates the impact
   *             of nondeterministic network delays in exchange for a larger but deterministic
   *             control latency.
   *             <p>This requires setting the ControlTimesyncFreqHz config in MotorOutputConfigs.
   *             Additionally, when this is enabled, the UpdateFreqHz of this request should be set
   *             to 0 Hz.
   *       </ul>
   * </ul>
   *
   * @param request Control object to request of the device
   * @return Code response of the request
   */
  public StatusCode setControl(DynamicMotionMagicDutyCycle request) {
    return talonFX.setControl(request);
  }

  /**
   * Requests Motion Magic® to target a final position using a motion profile. This dynamic request
   * allows runtime changes to Cruise Velocity, Acceleration, and Jerk. Users can optionally provide
   * a voltage feedforward. This control requires use of a CANivore.
   *
   * <p>Motion Magic® produces a motion profile in real-time while attempting to honor the specified
   * Cruise Velocity, Acceleration, and (optional) Jerk. This control mode does not use the Expo_kV
   * or Expo_kA configs.
   *
   * <p>Target position can be changed on-the-fly and Motion Magic® will do its best to adjust the
   * profile. This control mode is voltage-based, so relevant closed-loop gains will use Volts for
   * the numerator.
   *
   * <ul>
   *   <li><b>DynamicMotionMagicVoltage Parameters:</b>
   *       <ul>
   *         <li><b>Position:</b> Position to drive toward in rotations.
   *         <li><b>Velocity:</b> Cruise velocity for profiling. The signage does not matter as the
   *             device will use the absolute value for profile generation.
   *         <li><b>Acceleration:</b> Acceleration for profiling. The signage does not matter as the
   *             device will use the absolute value for profile generation.
   *         <li><b>Jerk:</b> Jerk for profiling. The signage does not matter as the device will use
   *             the absolute value for profile generation.
   *             <p>Jerk is optional; if this is set to zero, then Motion Magic® will not apply a
   *             Jerk limit.
   *         <li><b>EnableFOC:</b> Set to true to use FOC commutation (requires Phoenix Pro), which
   *             increases peak power by ~15%. Set to false to use trapezoidal commutation.
   *             <p>FOC improves motor performance by leveraging torque (current) control. However,
   *             this may be inconvenient for applications that require specifying duty cycle or
   *             voltage. CTR-Electronics has developed a hybrid method that combines the
   *             performances gains of FOC while still allowing applications to provide duty cycle
   *             or voltage demand. This not to be confused with simple sinusoidal control or phase
   *             voltage control which lacks the performance gains.
   *         <li><b>FeedForward:</b> Feedforward to apply in volts
   *         <li><b>Slot:</b> Select which gains are applied by selecting the slot. Use the
   *             configuration api to set the gain values for the selected slot before enabling this
   *             feature. Slot must be within [0,2].
   *         <li><b>OverrideBrakeDurNeutral:</b> Set to true to static-brake the rotor when output
   *             is zero (or within deadband). Set to false to use the NeutralMode configuration
   *             setting (default). This flag exists to provide the fundamental behavior of this
   *             control when output is zero, which is to provide 0V to the motor.
   *         <li><b>LimitForwardMotion:</b> Set to true to force forward limiting. This allows users
   *             to use other limit switch sensors connected to robot controller. This also allows
   *             use of active sensors that require external power.
   *         <li><b>LimitReverseMotion:</b> Set to true to force reverse limiting. This allows users
   *             to use other limit switch sensors connected to robot controller. This also allows
   *             use of active sensors that require external power.
   *         <li><b>IgnoreHardwareLimits:</b> Set to true to ignore hardware limit switches and the
   *             LimitForwardMotion and LimitReverseMotion parameters, instead allowing motion.
   *             <p>This can be useful on mechanisms such as an intake/feeder, where a limit switch
   *             stops motion while intaking but should be ignored when feeding to a shooter.
   *             <p>The hardware limit faults and Forward/ReverseLimit signals will still report the
   *             values of the limit switches regardless of this parameter.
   *         <li><b>UseTimesync:</b> Set to true to delay applying this control request until a
   *             timesync boundary (requires Phoenix Pro and CANivore). This eliminates the impact
   *             of nondeterministic network delays in exchange for a larger but deterministic
   *             control latency.
   *             <p>This requires setting the ControlTimesyncFreqHz config in MotorOutputConfigs.
   *             Additionally, when this is enabled, the UpdateFreqHz of this request should be set
   *             to 0 Hz.
   *       </ul>
   * </ul>
   *
   * @param request Control object to request of the device
   * @return Code response of the request
   */
  public StatusCode setControl(DynamicMotionMagicVoltage request) {
    return talonFX.setControl(request);
  }

  /**
   * Requests Motion Magic® to target a final position using a motion profile. This dynamic request
   * allows runtime changes to Cruise Velocity, Acceleration, and Jerk. Users can optionally provide
   * a torque current feedforward. This control requires use of a CANivore.
   *
   * <p>Motion Magic® produces a motion profile in real-time while attempting to honor the specified
   * Cruise Velocity, Acceleration, and (optional) Jerk. This control mode does not use the Expo_kV
   * or Expo_kA configs.
   *
   * <p>Target position can be changed on-the-fly and Motion Magic® will do its best to adjust the
   * profile. This control mode is based on torque current, so relevant closed-loop gains will use
   * Amperes for the numerator.
   *
   * <ul>
   *   <li><b>DynamicMotionMagicTorqueCurrentFOC Parameters:</b>
   *       <ul>
   *         <li><b>Position:</b> Position to drive toward in rotations.
   *         <li><b>Velocity:</b> Cruise velocity for profiling. The signage does not matter as the
   *             device will use the absolute value for profile generation.
   *         <li><b>Acceleration:</b> Acceleration for profiling. The signage does not matter as the
   *             device will use the absolute value for profile generation.
   *         <li><b>Jerk:</b> Jerk for profiling. The signage does not matter as the device will use
   *             the absolute value for profile generation.
   *             <p>Jerk is optional; if this is set to zero, then Motion Magic® will not apply a
   *             Jerk limit.
   *         <li><b>FeedForward:</b> Feedforward to apply in torque current in Amperes. User can use
   *             motor's kT to scale Newton-meter to Amperes.
   *         <li><b>Slot:</b> Select which gains are applied by selecting the slot. Use the
   *             configuration api to set the gain values for the selected slot before enabling this
   *             feature. Slot must be within [0,2].
   *         <li><b>OverrideCoastDurNeutral:</b> Set to true to coast the rotor when output is zero
   *             (or within deadband). Set to false to use the NeutralMode configuration setting
   *             (default). This flag exists to provide the fundamental behavior of this control
   *             when output is zero, which is to provide 0A (zero torque).
   *         <li><b>LimitForwardMotion:</b> Set to true to force forward limiting. This allows users
   *             to use other limit switch sensors connected to robot controller. This also allows
   *             use of active sensors that require external power.
   *         <li><b>LimitReverseMotion:</b> Set to true to force reverse limiting. This allows users
   *             to use other limit switch sensors connected to robot controller. This also allows
   *             use of active sensors that require external power.
   *         <li><b>IgnoreHardwareLimits:</b> Set to true to ignore hardware limit switches and the
   *             LimitForwardMotion and LimitReverseMotion parameters, instead allowing motion.
   *             <p>This can be useful on mechanisms such as an intake/feeder, where a limit switch
   *             stops motion while intaking but should be ignored when feeding to a shooter.
   *             <p>The hardware limit faults and Forward/ReverseLimit signals will still report the
   *             values of the limit switches regardless of this parameter.
   *         <li><b>UseTimesync:</b> Set to true to delay applying this control request until a
   *             timesync boundary (requires Phoenix Pro and CANivore). This eliminates the impact
   *             of nondeterministic network delays in exchange for a larger but deterministic
   *             control latency.
   *             <p>This requires setting the ControlTimesyncFreqHz config in MotorOutputConfigs.
   *             Additionally, when this is enabled, the UpdateFreqHz of this request should be set
   *             to 0 Hz.
   *       </ul>
   * </ul>
   *
   * @param request Control object to request of the device
   * @return Code response of the request
   */
  public StatusCode setControl(DynamicMotionMagicTorqueCurrentFOC request) {
    return talonFX.setControl(request);
  }

  /**
   * Differential control with duty cycle average target and position difference target.
   *
   * <ul>
   *   <li><b>Diff_DutyCycleOut_Position Parameters:</b>
   *       <ul>
   *         <li><b>AverageRequest:</b> Average DutyCycleOut request of the mechanism.
   *         <li><b>DifferentialRequest:</b> Differential PositionDutyCycle request of the
   *             mechanism.
   *       </ul>
   * </ul>
   *
   * @param request Control object to request of the device
   * @return Code response of the request
   */
  public StatusCode setControl(Diff_DutyCycleOut_Position request) {
    return talonFX.setControl(request);
  }

  /**
   * Differential control with position average target and position difference target using
   * dutycycle control.
   *
   * <ul>
   *   <li><b>Diff_PositionDutyCycle_Position Parameters:</b>
   *       <ul>
   *         <li><b>AverageRequest:</b> Average PositionDutyCycle request of the mechanism.
   *         <li><b>DifferentialRequest:</b> Differential PositionDutyCycle request of the
   *             mechanism.
   *       </ul>
   * </ul>
   *
   * @param request Control object to request of the device
   * @return Code response of the request
   */
  public StatusCode setControl(Diff_PositionDutyCycle_Position request) {
    return talonFX.setControl(request);
  }

  /**
   * Differential control with velocity average target and position difference target using
   * dutycycle control.
   *
   * <ul>
   *   <li><b>Diff_VelocityDutyCycle_Position Parameters:</b>
   *       <ul>
   *         <li><b>AverageRequest:</b> Average VelocityDutyCYcle request of the mechanism.
   *         <li><b>DifferentialRequest:</b> Differential PositionDutyCycle request of the
   *             mechanism.
   *       </ul>
   * </ul>
   *
   * @param request Control object to request of the device
   * @return Code response of the request
   */
  public StatusCode setControl(Diff_VelocityDutyCycle_Position request) {
    return talonFX.setControl(request);
  }

  /**
   * Differential control with Motion Magic® average target and position difference target using
   * dutycycle control.
   *
   * <ul>
   *   <li><b>Diff_MotionMagicDutyCycle_Position Parameters:</b>
   *       <ul>
   *         <li><b>AverageRequest:</b> Average MotionMagicDutyCycle request of the mechanism.
   *         <li><b>DifferentialRequest:</b> Differential PositionDutyCycle request of the
   *             mechanism.
   *       </ul>
   * </ul>
   *
   * @param request Control object to request of the device
   * @return Code response of the request
   */
  public StatusCode setControl(Diff_MotionMagicDutyCycle_Position request) {
    return talonFX.setControl(request);
  }

  /**
   * Differential control with duty cycle average target and velocity difference target.
   *
   * <ul>
   *   <li><b>Diff_DutyCycleOut_Velocity Parameters:</b>
   *       <ul>
   *         <li><b>AverageRequest:</b> Average DutyCycleOut request of the mechanism.
   *         <li><b>DifferentialRequest:</b> Differential VelocityDutyCycle request of the
   *             mechanism.
   *       </ul>
   * </ul>
   *
   * @param request Control object to request of the device
   * @return Code response of the request
   */
  public StatusCode setControl(Diff_DutyCycleOut_Velocity request) {
    return talonFX.setControl(request);
  }

  /**
   * Differential control with position average target and velocity difference target using
   * dutycycle control.
   *
   * <ul>
   *   <li><b>Diff_PositionDutyCycle_Velocity Parameters:</b>
   *       <ul>
   *         <li><b>AverageRequest:</b> Average PositionDutyCycle request of the mechanism.
   *         <li><b>DifferentialRequest:</b> Differential VelocityDutyCycle request of the
   *             mechanism.
   *       </ul>
   * </ul>
   *
   * @param request Control object to request of the device
   * @return Code response of the request
   */
  public StatusCode setControl(Diff_PositionDutyCycle_Velocity request) {
    return talonFX.setControl(request);
  }

  /**
   * Differential control with velocity average target and velocity difference target using
   * dutycycle control.
   *
   * <ul>
   *   <li><b>Diff_VelocityDutyCycle_Velocity Parameters:</b>
   *       <ul>
   *         <li><b>AverageRequest:</b> Average VelocityDutyCycle request of the mechanism.
   *         <li><b>DifferentialRequest:</b> Differential VelocityDutyCycle request of the
   *             mechanism.
   *       </ul>
   * </ul>
   *
   * @param request Control object to request of the device
   * @return Code response of the request
   */
  public StatusCode setControl(Diff_VelocityDutyCycle_Velocity request) {
    return talonFX.setControl(request);
  }

  /**
   * Differential control with Motion Magic® average target and velocity difference target using
   * dutycycle control.
   *
   * <ul>
   *   <li><b>Diff_MotionMagicDutyCycle_Velocity Parameters:</b>
   *       <ul>
   *         <li><b>AverageRequest:</b> Average MotionMagicDutyCycle request of the mechanism.
   *         <li><b>DifferentialRequest:</b> Differential VelocityDutyCycle request of the
   *             mechanism.
   *       </ul>
   * </ul>
   *
   * @param request Control object to request of the device
   * @return Code response of the request
   */
  public StatusCode setControl(Diff_MotionMagicDutyCycle_Velocity request) {
    return talonFX.setControl(request);
  }

  /**
   * Differential control with voltage average target and position difference target.
   *
   * <ul>
   *   <li><b>Diff_VoltageOut_Position Parameters:</b>
   *       <ul>
   *         <li><b>AverageRequest:</b> Average VoltageOut request of the mechanism.
   *         <li><b>DifferentialRequest:</b> Differential PositionVoltage request of the mechanism.
   *       </ul>
   * </ul>
   *
   * @param request Control object to request of the device
   * @return Code response of the request
   */
  public StatusCode setControl(Diff_VoltageOut_Position request) {
    return talonFX.setControl(request);
  }

  /**
   * Differential control with position average target and position difference target using voltage
   * control.
   *
   * <ul>
   *   <li><b>Diff_PositionVoltage_Position Parameters:</b>
   *       <ul>
   *         <li><b>AverageRequest:</b> Average PositionVoltage request of the mechanism.
   *         <li><b>DifferentialRequest:</b> Differential PositionVoltage request of the mechanism.
   *       </ul>
   * </ul>
   *
   * @param request Control object to request of the device
   * @return Code response of the request
   */
  public StatusCode setControl(Diff_PositionVoltage_Position request) {
    return talonFX.setControl(request);
  }

  /**
   * Differential control with velocity average target and position difference target using voltage
   * control.
   *
   * <ul>
   *   <li><b>Diff_VelocityVoltage_Position Parameters:</b>
   *       <ul>
   *         <li><b>AverageRequest:</b> Average VelocityVoltage request of the mechanism.
   *         <li><b>DifferentialRequest:</b> Differential PositionVoltage request of the mechanism.
   *       </ul>
   * </ul>
   *
   * @param request Control object to request of the device
   * @return Code response of the request
   */
  public StatusCode setControl(Diff_VelocityVoltage_Position request) {
    return talonFX.setControl(request);
  }

  /**
   * Differential control with Motion Magic® average target and position difference target using
   * voltage control.
   *
   * <ul>
   *   <li><b>Diff_MotionMagicVoltage_Position Parameters:</b>
   *       <ul>
   *         <li><b>AverageRequest:</b> Average MotionMagicVoltage request of the mechanism.
   *         <li><b>DifferentialRequest:</b> Differential PositionVoltage request of the mechanism.
   *       </ul>
   * </ul>
   *
   * @param request Control object to request of the device
   * @return Code response of the request
   */
  public StatusCode setControl(Diff_MotionMagicVoltage_Position request) {
    return talonFX.setControl(request);
  }

  /**
   * Differential control with voltage average target and velocity difference target.
   *
   * <ul>
   *   <li><b>Diff_VoltageOut_Velocity Parameters:</b>
   *       <ul>
   *         <li><b>AverageRequest:</b> Average VoltageOut request of the mechanism.
   *         <li><b>DifferentialRequest:</b> Differential VelocityVoltage request of the mechanism.
   *       </ul>
   * </ul>
   *
   * @param request Control object to request of the device
   * @return Code response of the request
   */
  public StatusCode setControl(Diff_VoltageOut_Velocity request) {
    return talonFX.setControl(request);
  }

  /**
   * Differential control with position average target and velocity difference target using voltage
   * control.
   *
   * <ul>
   *   <li><b>Diff_PositionVoltage_Velocity Parameters:</b>
   *       <ul>
   *         <li><b>AverageRequest:</b> Average PositionVoltage request of the mechanism.
   *         <li><b>DifferentialRequest:</b> Differential VelocityVoltage request of the mechanism.
   *       </ul>
   * </ul>
   *
   * @param request Control object to request of the device
   * @return Code response of the request
   */
  public StatusCode setControl(Diff_PositionVoltage_Velocity request) {
    return talonFX.setControl(request);
  }

  /**
   * Differential control with velocity average target and velocity difference target using voltage
   * control.
   *
   * <ul>
   *   <li><b>Diff_VelocityVoltage_Velocity Parameters:</b>
   *       <ul>
   *         <li><b>AverageRequest:</b> Average VelocityVoltage request of the mechanism.
   *         <li><b>DifferentialRequest:</b> Differential VelocityVoltage request of the mechanism.
   *       </ul>
   * </ul>
   *
   * @param request Control object to request of the device
   * @return Code response of the request
   */
  public StatusCode setControl(Diff_VelocityVoltage_Velocity request) {
    return talonFX.setControl(request);
  }

  /**
   * Differential control with Motion Magic® average target and velocity difference target using
   * voltage control.
   *
   * <ul>
   *   <li><b>Diff_MotionMagicVoltage_Velocity Parameters:</b>
   *       <ul>
   *         <li><b>AverageRequest:</b> Average MotionMagicVoltage request of the mechanism.
   *         <li><b>DifferentialRequest:</b> Differential VelocityVoltage request of the mechanism.
   *       </ul>
   * </ul>
   *
   * @param request Control object to request of the device
   * @return Code response of the request
   */
  public StatusCode setControl(Diff_MotionMagicVoltage_Velocity request) {
    return talonFX.setControl(request);
  }

  /**
   * Differential control with torque current average target and position difference target.
   *
   * <ul>
   *   <li><b>Diff_TorqueCurrentFOC_Position Parameters:</b>
   *       <ul>
   *         <li><b>AverageRequest:</b> Average TorqueCurrentFOC request of the mechanism.
   *         <li><b>DifferentialRequest:</b> Differential PositionTorqueCurrentFOC request of the
   *             mechanism.
   *       </ul>
   * </ul>
   *
   * @param request Control object to request of the device
   * @return Code response of the request
   */
  public StatusCode setControl(Diff_TorqueCurrentFOC_Position request) {
    return talonFX.setControl(request);
  }

  /**
   * Differential control with position average target and position difference target using torque
   * current control.
   *
   * <ul>
   *   <li><b>Diff_PositionTorqueCurrentFOC_Position Parameters:</b>
   *       <ul>
   *         <li><b>AverageRequest:</b> Average PositionTorqueCurrentFOC request of the mechanism.
   *         <li><b>DifferentialRequest:</b> Differential PositionTorqueCurrentFOC request of the
   *             mechanism.
   *       </ul>
   * </ul>
   *
   * @param request Control object to request of the device
   * @return Code response of the request
   */
  public StatusCode setControl(Diff_PositionTorqueCurrentFOC_Position request) {
    return talonFX.setControl(request);
  }

  /**
   * Differential control with velocity average target and position difference target using torque
   * current control.
   *
   * <ul>
   *   <li><b>Diff_VelocityTorqueCurrentFOC_Position Parameters:</b>
   *       <ul>
   *         <li><b>AverageRequest:</b> Average VelocityTorqueCurrentFOC request of the mechanism.
   *         <li><b>DifferentialRequest:</b> Differential PositionTorqueCurrentFOC request of the
   *             mechanism.
   *       </ul>
   * </ul>
   *
   * @param request Control object to request of the device
   * @return Code response of the request
   */
  public StatusCode setControl(Diff_VelocityTorqueCurrentFOC_Position request) {
    return talonFX.setControl(request);
  }

  /**
   * Differential control with Motion Magic® average target and position difference target using
   * torque current control.
   *
   * <ul>
   *   <li><b>Diff_MotionMagicTorqueCurrentFOC_Position Parameters:</b>
   *       <ul>
   *         <li><b>AverageRequest:</b> Average MotionMagicTorqueCurrentFOC request of the
   *             mechanism.
   *         <li><b>DifferentialRequest:</b> Differential PositionTorqueCurrentFOC request of the
   *             mechanism.
   *       </ul>
   * </ul>
   *
   * @param request Control object to request of the device
   * @return Code response of the request
   */
  public StatusCode setControl(Diff_MotionMagicTorqueCurrentFOC_Position request) {
    return talonFX.setControl(request);
  }

  /**
   * Differential control with torque current average target and velocity difference target.
   *
   * <ul>
   *   <li><b>Diff_TorqueCurrentFOC_Velocity Parameters:</b>
   *       <ul>
   *         <li><b>AverageRequest:</b> Average TorqueCurrentFOC request of the mechanism.
   *         <li><b>DifferentialRequest:</b> Differential VelocityTorqueCurrentFOC request of the
   *             mechanism.
   *       </ul>
   * </ul>
   *
   * @param request Control object to request of the device
   * @return Code response of the request
   */
  public StatusCode setControl(Diff_TorqueCurrentFOC_Velocity request) {
    return talonFX.setControl(request);
  }

  /**
   * Differential control with position average target and velocity difference target using torque
   * current control.
   *
   * <ul>
   *   <li><b>Diff_PositionTorqueCurrentFOC_Velocity Parameters:</b>
   *       <ul>
   *         <li><b>AverageRequest:</b> Average PositionTorqueCurrentFOC request of the mechanism.
   *         <li><b>DifferentialRequest:</b> Differential VelocityTorqueCurrentFOC request of the
   *             mechanism.
   *       </ul>
   * </ul>
   *
   * @param request Control object to request of the device
   * @return Code response of the request
   */
  public StatusCode setControl(Diff_PositionTorqueCurrentFOC_Velocity request) {
    return talonFX.setControl(request);
  }

  /**
   * Differential control with velocity average target and velocity difference target using torque
   * current control.
   *
   * <ul>
   *   <li><b>Diff_VelocityTorqueCurrentFOC_Velocity Parameters:</b>
   *       <ul>
   *         <li><b>AverageRequest:</b> Average VelocityTorqueCurrentFOC request of the mechanism.
   *         <li><b>DifferentialRequest:</b> Differential VelocityTorqueCurrentFOC request of the
   *             mechanism.
   *       </ul>
   * </ul>
   *
   * @param request Control object to request of the device
   * @return Code response of the request
   */
  public StatusCode setControl(Diff_VelocityTorqueCurrentFOC_Velocity request) {
    return talonFX.setControl(request);
  }

  /**
   * Differential control with Motion Magic® average target and velocity difference target using
   * torque current control.
   *
   * <ul>
   *   <li><b>Diff_MotionMagicTorqueCurrentFOC_Velocity Parameters:</b>
   *       <ul>
   *         <li><b>AverageRequest:</b> Average MotionMagicTorqueCurrentFOC request of the
   *             mechanism.
   *         <li><b>DifferentialRequest:</b> Differential VelocityTorqueCurrentFOC request of the
   *             mechanism.
   *       </ul>
   * </ul>
   *
   * @param request Control object to request of the device
   * @return Code response of the request
   */
  public StatusCode setControl(Diff_MotionMagicTorqueCurrentFOC_Velocity request) {
    return talonFX.setControl(request);
  }

  /**
   * Control device with generic control request object.
   *
   * <p>User must make sure the specified object is castable to a valid control request, otherwise
   * this function will fail at run-time and return the NotSupported StatusCode
   *
   * @param request Control object to request of the device
   * @return Status Code of the request, 0 is OK
   */
  public StatusCode setControl(ControlRequest request) {
    return talonFX.setControl(request);
  }

  /**
   * Sets the mechanism position of the device in mechanism rotations.
   *
   * <p>This will wait up to 0.100 seconds (100ms) by default.
   *
   * @param newValue Value to set to. Units are in rotations.
   * @return StatusCode of the set command
   */
  public StatusCode setPosition(double newValue) {
    return talonFX.setPosition(newValue);
  }

  /**
   * Sets the mechanism position of the device in mechanism rotations.
   *
   * @param newValue Value to set to. Units are in rotations.
   * @param timeoutSeconds Maximum time to wait up to in seconds.
   * @return StatusCode of the set command
   */
  public StatusCode setPosition(double newValue, double timeoutSeconds) {
    return talonFX.setPosition(newValue, timeoutSeconds);
  }

  /**
   * Sets the mechanism position of the device in mechanism rotations.
   *
   * <p>This will wait up to 0.100 seconds (100ms) by default.
   *
   * @param newValue Value to set to. Units are in rotations.
   * @return StatusCode of the set command
   */
  public StatusCode setPosition(Angle newValue) {
    return talonFX.setPosition(newValue);
  }

  /**
   * Sets the mechanism position of the device in mechanism rotations.
   *
   * @param newValue Value to set to. Units are in rotations.
   * @param timeoutSeconds Maximum time to wait up to in seconds.
   * @return StatusCode of the set command
   */
  public StatusCode setPosition(Angle newValue, double timeoutSeconds) {
    return talonFX.setPosition(newValue, timeoutSeconds);
  }

  /**
   * Clear the sticky faults in the device.
   *
   * <p>This typically has no impact on the device functionality. Instead, it just clears telemetry
   * faults that are accessible via API and Tuner Self-Test.
   *
   * <p>This will wait up to 0.100 seconds (100ms) by default.
   *
   * @return StatusCode of the set command
   */
  public StatusCode clearStickyFaults() {
    return talonFX.clearStickyFaults();
  }

  /**
   * Clear the sticky faults in the device.
   *
   * <p>This typically has no impact on the device functionality. Instead, it just clears telemetry
   * faults that are accessible via API and Tuner Self-Test.
   *
   * @param timeoutSeconds Maximum time to wait up to in seconds.
   * @return StatusCode of the set command
   */
  public StatusCode clearStickyFaults(double timeoutSeconds) {
    return talonFX.clearStickyFaults(timeoutSeconds);
  }

  /**
   * Clear sticky fault: Hardware fault occurred
   *
   * <p>This will wait up to 0.100 seconds (100ms) by default.
   *
   * @return StatusCode of the set command
   */
  public StatusCode clearStickyFault_Hardware() {
    return talonFX.clearStickyFault_Hardware();
  }

  /**
   * Clear sticky fault: Hardware fault occurred
   *
   * @param timeoutSeconds Maximum time to wait up to in seconds.
   * @return StatusCode of the set command
   */
  public StatusCode clearStickyFault_Hardware(double timeoutSeconds) {
    return talonFX.clearStickyFault_Hardware(timeoutSeconds);
  }

  /**
   * Clear sticky fault: Processor temperature exceeded limit
   *
   * <p>This will wait up to 0.100 seconds (100ms) by default.
   *
   * @return StatusCode of the set command
   */
  public StatusCode clearStickyFault_ProcTemp() {
    return talonFX.clearStickyFault_ProcTemp();
  }

  /**
   * Clear sticky fault: Processor temperature exceeded limit
   *
   * @param timeoutSeconds Maximum time to wait up to in seconds.
   * @return StatusCode of the set command
   */
  public StatusCode clearStickyFault_ProcTemp(double timeoutSeconds) {
    return talonFX.clearStickyFault_ProcTemp(timeoutSeconds);
  }

  /**
   * Clear sticky fault: Device temperature exceeded limit
   *
   * <p>This will wait up to 0.100 seconds (100ms) by default.
   *
   * @return StatusCode of the set command
   */
  public StatusCode clearStickyFault_DeviceTemp() {
    return talonFX.clearStickyFault_DeviceTemp();
  }

  /**
   * Clear sticky fault: Device temperature exceeded limit
   *
   * @param timeoutSeconds Maximum time to wait up to in seconds.
   * @return StatusCode of the set command
   */
  public StatusCode clearStickyFault_DeviceTemp(double timeoutSeconds) {
    return talonFX.clearStickyFault_DeviceTemp(timeoutSeconds);
  }

  /**
   * Clear sticky fault: Device supply voltage dropped to near brownout levels
   *
   * <p>This will wait up to 0.100 seconds (100ms) by default.
   *
   * @return StatusCode of the set command
   */
  public StatusCode clearStickyFault_Undervoltage() {
    return talonFX.clearStickyFault_Undervoltage();
  }

  /**
   * Clear sticky fault: Device supply voltage dropped to near brownout levels
   *
   * @param timeoutSeconds Maximum time to wait up to in seconds.
   * @return StatusCode of the set command
   */
  public StatusCode clearStickyFault_Undervoltage(double timeoutSeconds) {
    return talonFX.clearStickyFault_Undervoltage(timeoutSeconds);
  }

  /**
   * Clear sticky fault: Device boot while detecting the enable signal
   *
   * <p>This will wait up to 0.100 seconds (100ms) by default.
   *
   * @return StatusCode of the set command
   */
  public StatusCode clearStickyFault_BootDuringEnable() {
    return talonFX.clearStickyFault_BootDuringEnable();
  }

  /**
   * Clear sticky fault: Device boot while detecting the enable signal
   *
   * @param timeoutSeconds Maximum time to wait up to in seconds.
   * @return StatusCode of the set command
   */
  public StatusCode clearStickyFault_BootDuringEnable(double timeoutSeconds) {
    return talonFX.clearStickyFault_BootDuringEnable(timeoutSeconds);
  }

  /**
   * Clear sticky fault: An unlicensed feature is in use, device may not behave as expected.
   *
   * <p>This will wait up to 0.100 seconds (100ms) by default.
   *
   * @return StatusCode of the set command
   */
  public StatusCode clearStickyFault_UnlicensedFeatureInUse() {
    return talonFX.clearStickyFault_UnlicensedFeatureInUse();
  }

  /**
   * Clear sticky fault: An unlicensed feature is in use, device may not behave as expected.
   *
   * @param timeoutSeconds Maximum time to wait up to in seconds.
   * @return StatusCode of the set command
   */
  public StatusCode clearStickyFault_UnlicensedFeatureInUse(double timeoutSeconds) {
    return talonFX.clearStickyFault_UnlicensedFeatureInUse(timeoutSeconds);
  }

  /**
   * Clear sticky fault: Bridge was disabled most likely due to supply voltage dropping too low.
   *
   * <p>This will wait up to 0.100 seconds (100ms) by default.
   *
   * @return StatusCode of the set command
   */
  public StatusCode clearStickyFault_BridgeBrownout() {
    return talonFX.clearStickyFault_BridgeBrownout();
  }

  /**
   * Clear sticky fault: Bridge was disabled most likely due to supply voltage dropping too low.
   *
   * @param timeoutSeconds Maximum time to wait up to in seconds.
   * @return StatusCode of the set command
   */
  public StatusCode clearStickyFault_BridgeBrownout(double timeoutSeconds) {
    return talonFX.clearStickyFault_BridgeBrownout(timeoutSeconds);
  }

  /**
   * Clear sticky fault: The remote sensor has reset.
   *
   * <p>This will wait up to 0.100 seconds (100ms) by default.
   *
   * @return StatusCode of the set command
   */
  public StatusCode clearStickyFault_RemoteSensorReset() {
    return talonFX.clearStickyFault_RemoteSensorReset();
  }

  /**
   * Clear sticky fault: The remote sensor has reset.
   *
   * @param timeoutSeconds Maximum time to wait up to in seconds.
   * @return StatusCode of the set command
   */
  public StatusCode clearStickyFault_RemoteSensorReset(double timeoutSeconds) {
    return talonFX.clearStickyFault_RemoteSensorReset(timeoutSeconds);
  }

  /**
   * Clear sticky fault: The remote Talon used for differential control is not present on CAN Bus.
   *
   * <p>This will wait up to 0.100 seconds (100ms) by default.
   *
   * @return StatusCode of the set command
   */
  public StatusCode clearStickyFault_MissingDifferentialFX() {
    return talonFX.clearStickyFault_MissingDifferentialFX();
  }

  /**
   * Clear sticky fault: The remote Talon used for differential control is not present on CAN Bus.
   *
   * @param timeoutSeconds Maximum time to wait up to in seconds.
   * @return StatusCode of the set command
   */
  public StatusCode clearStickyFault_MissingDifferentialFX(double timeoutSeconds) {
    return talonFX.clearStickyFault_MissingDifferentialFX(timeoutSeconds);
  }

  /**
   * Clear sticky fault: The remote sensor position has overflowed. Because of the nature of remote
   * sensors, it is possible for the remote sensor position to overflow beyond what is supported by
   * the status signal frame. However, this is rare and cannot occur over the course of an FRC match
   * under normal use.
   *
   * <p>This will wait up to 0.100 seconds (100ms) by default.
   *
   * @return StatusCode of the set command
   */
  public StatusCode clearStickyFault_RemoteSensorPosOverflow() {
    return talonFX.clearStickyFault_RemoteSensorPosOverflow();
  }

  /**
   * Clear sticky fault: The remote sensor position has overflowed. Because of the nature of remote
   * sensors, it is possible for the remote sensor position to overflow beyond what is supported by
   * the status signal frame. However, this is rare and cannot occur over the course of an FRC match
   * under normal use.
   *
   * @param timeoutSeconds Maximum time to wait up to in seconds.
   * @return StatusCode of the set command
   */
  public StatusCode clearStickyFault_RemoteSensorPosOverflow(double timeoutSeconds) {
    return talonFX.clearStickyFault_RemoteSensorPosOverflow(timeoutSeconds);
  }

  /**
   * Clear sticky fault: Supply Voltage has exceeded the maximum voltage rating of device.
   *
   * <p>This will wait up to 0.100 seconds (100ms) by default.
   *
   * @return StatusCode of the set command
   */
  public StatusCode clearStickyFault_OverSupplyV() {
    return talonFX.clearStickyFault_OverSupplyV();
  }

  /**
   * Clear sticky fault: Supply Voltage has exceeded the maximum voltage rating of device.
   *
   * @param timeoutSeconds Maximum time to wait up to in seconds.
   * @return StatusCode of the set command
   */
  public StatusCode clearStickyFault_OverSupplyV(double timeoutSeconds) {
    return talonFX.clearStickyFault_OverSupplyV(timeoutSeconds);
  }

  /**
   * Clear sticky fault: Supply Voltage is unstable. Ensure you are using a battery and current
   * limited power supply.
   *
   * <p>This will wait up to 0.100 seconds (100ms) by default.
   *
   * @return StatusCode of the set command
   */
  public StatusCode clearStickyFault_UnstableSupplyV() {
    return talonFX.clearStickyFault_UnstableSupplyV();
  }

  /**
   * Clear sticky fault: Supply Voltage is unstable. Ensure you are using a battery and current
   * limited power supply.
   *
   * @param timeoutSeconds Maximum time to wait up to in seconds.
   * @return StatusCode of the set command
   */
  public StatusCode clearStickyFault_UnstableSupplyV(double timeoutSeconds) {
    return talonFX.clearStickyFault_UnstableSupplyV(timeoutSeconds);
  }

  /**
   * Clear sticky fault: Reverse limit switch has been asserted. Output is set to neutral.
   *
   * <p>This will wait up to 0.100 seconds (100ms) by default.
   *
   * @return StatusCode of the set command
   */
  public StatusCode clearStickyFault_ReverseHardLimit() {
    return talonFX.clearStickyFault_ReverseHardLimit();
  }

  /**
   * Clear sticky fault: Reverse limit switch has been asserted. Output is set to neutral.
   *
   * @param timeoutSeconds Maximum time to wait up to in seconds.
   * @return StatusCode of the set command
   */
  public StatusCode clearStickyFault_ReverseHardLimit(double timeoutSeconds) {
    return talonFX.clearStickyFault_ReverseHardLimit(timeoutSeconds);
  }

  /**
   * Clear sticky fault: Forward limit switch has been asserted. Output is set to neutral.
   *
   * <p>This will wait up to 0.100 seconds (100ms) by default.
   *
   * @return StatusCode of the set command
   */
  public StatusCode clearStickyFault_ForwardHardLimit() {
    return talonFX.clearStickyFault_ForwardHardLimit();
  }

  /**
   * Clear sticky fault: Forward limit switch has been asserted. Output is set to neutral.
   *
   * @param timeoutSeconds Maximum time to wait up to in seconds.
   * @return StatusCode of the set command
   */
  public StatusCode clearStickyFault_ForwardHardLimit(double timeoutSeconds) {
    return talonFX.clearStickyFault_ForwardHardLimit(timeoutSeconds);
  }

  /**
   * Clear sticky fault: Reverse soft limit has been asserted. Output is set to neutral.
   *
   * <p>This will wait up to 0.100 seconds (100ms) by default.
   *
   * @return StatusCode of the set command
   */
  public StatusCode clearStickyFault_ReverseSoftLimit() {
    return talonFX.clearStickyFault_ReverseSoftLimit();
  }

  /**
   * Clear sticky fault: Reverse soft limit has been asserted. Output is set to neutral.
   *
   * @param timeoutSeconds Maximum time to wait up to in seconds.
   * @return StatusCode of the set command
   */
  public StatusCode clearStickyFault_ReverseSoftLimit(double timeoutSeconds) {
    return talonFX.clearStickyFault_ReverseSoftLimit(timeoutSeconds);
  }

  /**
   * Clear sticky fault: Forward soft limit has been asserted. Output is set to neutral.
   *
   * <p>This will wait up to 0.100 seconds (100ms) by default.
   *
   * @return StatusCode of the set command
   */
  public StatusCode clearStickyFault_ForwardSoftLimit() {
    return talonFX.clearStickyFault_ForwardSoftLimit();
  }

  /**
   * Clear sticky fault: Forward soft limit has been asserted. Output is set to neutral.
   *
   * @param timeoutSeconds Maximum time to wait up to in seconds.
   * @return StatusCode of the set command
   */
  public StatusCode clearStickyFault_ForwardSoftLimit(double timeoutSeconds) {
    return talonFX.clearStickyFault_ForwardSoftLimit(timeoutSeconds);
  }

  /**
   * Clear sticky fault: The remote soft limit device is not present on CAN Bus.
   *
   * <p>This will wait up to 0.100 seconds (100ms) by default.
   *
   * @return StatusCode of the set command
   */
  public StatusCode clearStickyFault_MissingSoftLimitRemote() {
    return talonFX.clearStickyFault_MissingSoftLimitRemote();
  }

  /**
   * Clear sticky fault: The remote soft limit device is not present on CAN Bus.
   *
   * @param timeoutSeconds Maximum time to wait up to in seconds.
   * @return StatusCode of the set command
   */
  public StatusCode clearStickyFault_MissingSoftLimitRemote(double timeoutSeconds) {
    return talonFX.clearStickyFault_MissingSoftLimitRemote(timeoutSeconds);
  }

  /**
   * Clear sticky fault: The remote limit switch device is not present on CAN Bus.
   *
   * <p>This will wait up to 0.100 seconds (100ms) by default.
   *
   * @return StatusCode of the set command
   */
  public StatusCode clearStickyFault_MissingHardLimitRemote() {
    return talonFX.clearStickyFault_MissingHardLimitRemote();
  }

  /**
   * Clear sticky fault: The remote limit switch device is not present on CAN Bus.
   *
   * @param timeoutSeconds Maximum time to wait up to in seconds.
   * @return StatusCode of the set command
   */
  public StatusCode clearStickyFault_MissingHardLimitRemote(double timeoutSeconds) {
    return talonFX.clearStickyFault_MissingHardLimitRemote(timeoutSeconds);
  }

  /**
   * Clear sticky fault: The remote sensor's data is no longer trusted. This can happen if the
   * remote sensor disappears from the CAN bus or if the remote sensor indicates its data is no
   * longer valid, such as when a CANcoder's magnet strength falls into the "red" range.
   *
   * <p>This will wait up to 0.100 seconds (100ms) by default.
   *
   * @return StatusCode of the set command
   */
  public StatusCode clearStickyFault_RemoteSensorDataInvalid() {
    return talonFX.clearStickyFault_RemoteSensorDataInvalid();
  }

  /**
   * Clear sticky fault: The remote sensor's data is no longer trusted. This can happen if the
   * remote sensor disappears from the CAN bus or if the remote sensor indicates its data is no
   * longer valid, such as when a CANcoder's magnet strength falls into the "red" range.
   *
   * @param timeoutSeconds Maximum time to wait up to in seconds.
   * @return StatusCode of the set command
   */
  public StatusCode clearStickyFault_RemoteSensorDataInvalid(double timeoutSeconds) {
    return talonFX.clearStickyFault_RemoteSensorDataInvalid(timeoutSeconds);
  }

  /**
   * Clear sticky fault: The remote sensor used for fusion has fallen out of sync to the local
   * sensor. A re-synchronization has occurred, which may cause a discontinuity. This typically
   * happens if there is significant slop in the mechanism, or if the RotorToSensorRatio
   * configuration parameter is incorrect.
   *
   * <p>This will wait up to 0.100 seconds (100ms) by default.
   *
   * @return StatusCode of the set command
   */
  public StatusCode clearStickyFault_FusedSensorOutOfSync() {
    return talonFX.clearStickyFault_FusedSensorOutOfSync();
  }

  /**
   * Clear sticky fault: The remote sensor used for fusion has fallen out of sync to the local
   * sensor. A re-synchronization has occurred, which may cause a discontinuity. This typically
   * happens if there is significant slop in the mechanism, or if the RotorToSensorRatio
   * configuration parameter is incorrect.
   *
   * @param timeoutSeconds Maximum time to wait up to in seconds.
   * @return StatusCode of the set command
   */
  public StatusCode clearStickyFault_FusedSensorOutOfSync(double timeoutSeconds) {
    return talonFX.clearStickyFault_FusedSensorOutOfSync(timeoutSeconds);
  }

  /**
   * Clear sticky fault: Stator current limit occured.
   *
   * <p>This will wait up to 0.100 seconds (100ms) by default.
   *
   * @return StatusCode of the set command
   */
  public StatusCode clearStickyFault_StatorCurrLimit() {
    return talonFX.clearStickyFault_StatorCurrLimit();
  }

  /**
   * Clear sticky fault: Stator current limit occured.
   *
   * @param timeoutSeconds Maximum time to wait up to in seconds.
   * @return StatusCode of the set command
   */
  public StatusCode clearStickyFault_StatorCurrLimit(double timeoutSeconds) {
    return talonFX.clearStickyFault_StatorCurrLimit(timeoutSeconds);
  }

  /**
   * Clear sticky fault: Supply current limit occured.
   *
   * <p>This will wait up to 0.100 seconds (100ms) by default.
   *
   * @return StatusCode of the set command
   */
  public StatusCode clearStickyFault_SupplyCurrLimit() {
    return talonFX.clearStickyFault_SupplyCurrLimit();
  }

  /**
   * Clear sticky fault: Supply current limit occured.
   *
   * @param timeoutSeconds Maximum time to wait up to in seconds.
   * @return StatusCode of the set command
   */
  public StatusCode clearStickyFault_SupplyCurrLimit(double timeoutSeconds) {
    return talonFX.clearStickyFault_SupplyCurrLimit(timeoutSeconds);
  }

  /**
   * Clear sticky fault: Using Fused CANcoder feature while unlicensed. Device has fallen back to
   * remote CANcoder.
   *
   * <p>This will wait up to 0.100 seconds (100ms) by default.
   *
   * @return StatusCode of the set command
   */
  public StatusCode clearStickyFault_UsingFusedCANcoderWhileUnlicensed() {
    return talonFX.clearStickyFault_UsingFusedCANcoderWhileUnlicensed();
  }

  /**
   * Clear sticky fault: Using Fused CANcoder feature while unlicensed. Device has fallen back to
   * remote CANcoder.
   *
   * @param timeoutSeconds Maximum time to wait up to in seconds.
   * @return StatusCode of the set command
   */
  public StatusCode clearStickyFault_UsingFusedCANcoderWhileUnlicensed(double timeoutSeconds) {
    return talonFX.clearStickyFault_UsingFusedCANcoderWhileUnlicensed(timeoutSeconds);
  }

  /**
   * Clear sticky fault: Static brake was momentarily disabled due to excessive braking current
   * while disabled.
   *
   * <p>This will wait up to 0.100 seconds (100ms) by default.
   *
   * @return StatusCode of the set command
   */
  public StatusCode clearStickyFault_StaticBrakeDisabled() {
    return talonFX.clearStickyFault_StaticBrakeDisabled();
  }

  /**
   * Clear sticky fault: Static brake was momentarily disabled due to excessive braking current
   * while disabled.
   *
   * @param timeoutSeconds Maximum time to wait up to in seconds.
   * @return StatusCode of the set command
   */
  public StatusCode clearStickyFault_StaticBrakeDisabled(double timeoutSeconds) {
    return talonFX.clearStickyFault_StaticBrakeDisabled(timeoutSeconds);
  }

  /**
   * Constructs a new Talon FX motor controller object.
   *
   * @param deviceId ID of the device, as configured in Phoenix Tuner.
   * @param canbus Name of the CAN bus this device is on. Possible CAN bus strings are:
   *     <ul>
   *       <li>"rio" for the native roboRIO CAN bus
   *       <li>CANivore name or serial number
   *       <li>SocketCAN interface (non-FRC Linux only)
   *       <li>"*" for any CANivore seen by the program
   *       <li>empty string (default) to select the default for the system:
   *           <ul>
   *             <li>"rio" on roboRIO
   *             <li>"can0" on Linux
   *             <li>"*" on Windows
   *           </ul>
   *     </ul>
   */
  public RSTalonFX(int deviceId, String canbus) {
    talonFX = new TalonFX(deviceId, canbus);
    init();
  }

  public void close() {
    talonFX.close();
  }

  /**
   * Common interface for setting the speed of a motor controller.
   *
   * @param speed The speed to set. Value should be between -1.0 and 1.0.
   */
  public void set(double speed) {
    talonFX.set(speed);
  }

  /**
   * Common interface for seting the direct voltage output of a motor controller.
   *
   * @param volts The voltage to output.
   */
  public void setVoltage(double volts) {
    talonFX.setVoltage(volts);
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
  @Override
  public void setVoltage(Voltage outputVoltage) {
    RSGenericMotorController.super.setVoltage(outputVoltage);
  }

  /**
   * Common interface for getting the current set speed of a motor controller.
   *
   * @return The current set speed. Value is between -1.0 and 1.0.
   */
  public double get() {
    return talonFX.get();
  }

  /**
   * Common interface for inverting direction of a motor controller.
   *
   * @param isInverted The state of inversion true is inverted.
   */
  @Override
  public void setInverted(boolean isInverted) {
    inverted = isInverted;
    getConfigurator()
        .apply(
            new MotorOutputConfigs()
                .withInverted(
                    isInverted
                        ? InvertedValue.Clockwise_Positive
                        : InvertedValue.CounterClockwise_Positive));
  }

  /**
   * Common interface for returning if a motor controller is in the inverted state or not.
   *
   * @return isInverted The state of the inversion true is inverted.
   */
  @Override
  public boolean getInverted() {
    return inverted;
  }

  /**
   * @return The device ID of this device [0,62].
   */
  public int getDeviceID() {
    return talonFX.getDeviceID();
  }

  /**
   * @return Name of the network this device is on.
   */
  public String getNetwork() {
    return talonFX.getNetwork();
  }

  /**
   * Gets a number unique for this device's hardware type and ID. This number is not unique across
   * networks.
   *
   * <p>This can be used to easily reference hardware devices on the same network in collections
   * such as maps.
   *
   * @return Hash of this device.
   */
  public long getDeviceHash() {
    return talonFX.getDeviceHash();
  }

  /**
   * Get the latest applied control. Caller can cast this to the derived class if they know its
   * type. Otherwise, use {@link ControlRequest#getControlInfo} to get info out of it.
   *
   * @return Latest applied control
   */
  public ControlRequest getAppliedControl() {
    return talonFX.getAppliedControl();
  }

  /**
   * @return true if device has reset since the previous call of this routine.
   */
  public boolean hasResetOccurred() {
    return talonFX.hasResetOccurred();
  }

  /**
   * @return a {@link BooleanSupplier} that checks for device resets.
   */
  public BooleanSupplier getResetOccurredChecker() {
    return talonFX.getResetOccurredChecker();
  }

  /**
   * Returns whether the device is still connected to the robot. This is equivalent to refreshing
   * and checking the latency of the Version status signal.
   *
   * <p>This defaults to reporting a device as disconnected if it has not been seen for over 0.5
   * seconds.
   *
   * @return true if the device is connected
   */
  public boolean isConnected() {
    return talonFX.isConnected();
  }

  /**
   * Returns whether the device is still connected to the robot. This is equivalent to refreshing
   * and checking the latency of the Version status signal.
   *
   * @param maxLatencySeconds The maximum latency of the Version status signal before the device is
   *     reported as disconnected
   * @return true if the device is connected
   */
  public boolean isConnected(double maxLatencySeconds) {
    return talonFX.isConnected(maxLatencySeconds);
  }

  /**
   * Optimizes the device's bus utilization by reducing the update frequencies of its status
   * signals.
   *
   * <p>All status signals that have not been explicitly given an update frequency using {@link
   * BaseStatusSignal#setUpdateFrequency} will be disabled. Note that if other status signals in the
   * same status frame have been given an update frequency, the update frequency will be honored for
   * the entire frame.
   *
   * <p>This function only needs to be called once in the robot program. Additionally, this method
   * does not necessarily need to be called after setting the update frequencies of other signals.
   *
   * <p>To restore the default status update frequencies, call {@link #resetSignalFrequencies}.
   * Alternatively, remove this method call, redeploy the robot application, and power-cycle the
   * device on the bus. The user can also override individual status update frequencies using {@link
   * BaseStatusSignal#setUpdateFrequency}.
   *
   * <p>This will wait up to 0.100 seconds (100ms) for each signal by default.
   *
   * @return Status code of the first failed update frequency set call, or OK if all succeeded
   */
  public StatusCode optimizeBusUtilization() {
    return talonFX.optimizeBusUtilization();
  }

  /**
   * Optimizes the device's bus utilization by reducing the update frequencies of its status
   * signals.
   *
   * <p>All status signals that have not been explicitly given an update frequency using {@link
   * BaseStatusSignal#setUpdateFrequency} will be disabled. Note that if other status signals in the
   * same status frame have been given an update frequency, the update frequency will be honored for
   * the entire frame.
   *
   * <p>This function only needs to be called once on this device in the robot program.
   * Additionally, this method does not necessarily need to be called after setting the update
   * frequencies of other signals.
   *
   * <p>To restore the default status update frequencies, call {@link #resetSignalFrequencies}.
   * Alternatively, remove this method call, redeploy the robot application, and power-cycle the
   * device on the bus. The user can also override individual status update frequencies using {@link
   * BaseStatusSignal#setUpdateFrequency}.
   *
   * @param optimizedFreqHz The update frequency to apply to the optimized status signals. A
   *     frequency of 0 Hz (default) will turn off the signals. Otherwise, the minimum supported
   *     signal frequency is 4 Hz.
   * @param timeoutSeconds Maximum amount of time to wait for each status frame when performing the
   *     action
   * @return Status code of the first failed update frequency set call, or OK if all succeeded
   */
  public StatusCode optimizeBusUtilization(double optimizedFreqHz, double timeoutSeconds) {
    return talonFX.optimizeBusUtilization(optimizedFreqHz, timeoutSeconds);
  }

  /**
   * Resets the update frequencies of all the device's status signals to the defaults.
   *
   * <p>This restores the default update frequency of all status signals, including status signals
   * explicitly given an update frequency using {@link BaseStatusSignal#setUpdateFrequency} and
   * status signals optimized out using {@link #optimizeBusUtilization}.
   *
   * <p>This will wait up to 0.100 seconds (100ms) for each signal by default.
   *
   * @return Status code of the first failed update frequency set call, or OK if all succeeded
   */
  public StatusCode resetSignalFrequencies() {
    return talonFX.resetSignalFrequencies();
  }

  /**
   * Resets the update frequencies of all the device's status signals to the defaults.
   *
   * <p>This restores the default update frequency of all status signals, including status signals
   * explicitly given an update frequency using {@link BaseStatusSignal#setUpdateFrequency} and
   * status signals optimized out using {@link #optimizeBusUtilization}.
   *
   * @param timeoutSeconds Maximum amount of time to wait for each status frame when performing the
   *     action
   * @return Status code of the first failed update frequency set call, or OK if all succeeded
   */
  public StatusCode resetSignalFrequencies(double timeoutSeconds) {
    return talonFX.resetSignalFrequencies(timeoutSeconds);
  }

  /** Common interface for disabling a motor controller. */
  public void disable() {
    talonFX.disable();
  }

  /** Common interface to stop motor movement until set is called again. */
  public void stopMotor() {
    talonFX.stopMotor();
  }

  @Override
  public void setNeutralMode(NeutralMode mode) {
    switch (mode) {
      case BRAKE:
        this.setNeutralMode(NeutralModeValue.Brake);
        break;
      case COAST:
        this.setNeutralMode(NeutralModeValue.Coast);
        break;
    }
  }

  /**
   * Sets the mode of operation when output is neutral or disabled.
   *
   * <p>Since neutral mode is a config, this API is blocking. We recommend that users avoid calling
   * this API periodically.
   *
   * <p>This will wait up to 0.100 seconds (100ms) by default.
   *
   * @param neutralMode The state of the motor controller bridge when output is neutral or disabled
   * @return Status of refreshing and applying the neutral mode config
   */
  public StatusCode setNeutralMode(NeutralModeValue neutralMode) {
    return talonFX.setNeutralMode(neutralMode);
  }

  /**
   * Sets the mode of operation when output is neutral or disabled.
   *
   * <p>Since neutral mode is a config, this API is blocking. We recommend that users avoid calling
   * this API periodically.
   *
   * @param neutralMode The state of the motor controller bridge when output is neutral or disabled
   * @param timeoutSeconds Maximum amount of time to wait when performing configuration
   * @return Status of refreshing and applying the neutral mode config
   */
  public StatusCode setNeutralMode(NeutralModeValue neutralMode, double timeoutSeconds) {
    return talonFX.setNeutralMode(neutralMode, timeoutSeconds);
  }

  public void initSendable(SendableBuilder builder) {
    builder.setSmartDashboardType("RSTalonFX");
    builder.addDoubleProperty("VelocityRPS", () -> this.getVelocity().getValueAsDouble(), null);
    builder.addDoubleProperty(
        "PositionRotations", () -> this.getPosition().getValueAsDouble(), null);
    builder.addDoubleProperty(
        "StatorCurrentAmps", () -> this.getStatorCurrent().getValueAsDouble(), null);
  }

  /**
   * @return Description of motor controller
   */
  public String getDescription() {
    return talonFX.getDescription();
  }

  /**
   * Feed the motor safety object.
   *
   * <p>Resets the timer on this object that is used to do the timeouts.
   */
  public void feed() {
    talonFX.feed();
  }

  /**
   * Set the expiration time for the corresponding motor safety object.
   *
   * @param expirationTime The timeout value in seconds.
   */
  public void setExpiration(double expirationTime) {
    talonFX.setExpiration(expirationTime);
  }

  /**
   * Retrieve the timeout value for the corresponding motor safety object.
   *
   * @return the timeout value in seconds.
   */
  public double getExpiration() {
    return talonFX.getExpiration();
  }

  /**
   * Determine of the motor is still operating or has timed out.
   *
   * @return a true value if the motor is still operating normally and hasn't timed out.
   */
  public boolean isAlive() {
    return talonFX.isAlive();
  }

  /**
   * Enable/disable motor safety for this device.
   *
   * <p>Turn on and off the motor safety option for this object.
   *
   * @param enabled True if motor safety is enforced for this object.
   */
  public void setSafetyEnabled(boolean enabled) {
    talonFX.setSafetyEnabled(enabled);
  }

  /**
   * Return the state of the motor safety enabled flag.
   *
   * <p>Return if the motor safety is currently enabled for this device.
   *
   * @return True if motor safety is enforced for this device
   */
  public boolean isSafetyEnabled() {
    return talonFX.isSafetyEnabled();
  }

  /**
   * Runs the motor given a speed value in the clockwise direction. This ignores inverts. A negative
   * value may be inputted, which will run counterclockwise.
   *
   * @param value a value between -1 and 1 to run the motor in.
   */
  public void setSpeedClockwise(double value) {
    var multiplier = inverted ? 1.0F : -1.0F;
    set(multiplier * value);
  }

  public void simUpdate() {
    simMotor.setState(
        talonFX.getPosition().getValue().in(Radians),
        talonFX.getVelocity().getValue().in(RadiansPerSecond));
  }
}
