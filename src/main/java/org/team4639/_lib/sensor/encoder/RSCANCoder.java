package org.team4639._lib.sensor.encoder;

import com.ctre.phoenix6.CANBus;
import com.ctre.phoenix6.StatusCode;
import com.ctre.phoenix6.StatusSignal;
import com.ctre.phoenix6.configs.CANcoderConfigurator;
import com.ctre.phoenix6.controls.ControlRequest;
import com.ctre.phoenix6.hardware.CANcoder;
import com.ctre.phoenix6.signals.MagnetHealthValue;
import com.ctre.phoenix6.sim.CANcoderSimState;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.units.measure.Voltage;
import edu.wpi.first.util.sendable.SendableBuilder;
import java.util.function.BooleanSupplier;

public class RSCANCoder {
  protected CANcoder cancoder;

  public RSCANCoder(int deviceId) {
    cancoder = new CANcoder(deviceId);
  }

  public RSCANCoder(int deviceId, CANBus canbus) {
    cancoder = new CANcoder(deviceId, canbus);
  }

  public CANcoderConfigurator getConfigurator() {
    return cancoder.getConfigurator();
  }

  public CANcoderSimState getSimState() {
    return cancoder.getSimState();
  }

  public StatusSignal<Integer> getVersionMajor() {
    return cancoder.getVersionMajor();
  }

  public StatusSignal<Integer> getVersionMajor(boolean refresh) {
    return cancoder.getVersionMajor(refresh);
  }

  public StatusSignal<Integer> getVersionMinor() {
    return cancoder.getVersionMinor();
  }

  public StatusSignal<Integer> getVersionMinor(boolean refresh) {
    return cancoder.getVersionMinor(refresh);
  }

  public StatusSignal<Integer> getVersionBugfix() {
    return cancoder.getVersionBugfix();
  }

  public StatusSignal<Integer> getVersionBugfix(boolean refresh) {
    return cancoder.getVersionBugfix(refresh);
  }

  public StatusSignal<Integer> getVersionBuild() {
    return cancoder.getVersionBuild();
  }

  public StatusSignal<Integer> getVersionBuild(boolean refresh) {
    return cancoder.getVersionBuild(refresh);
  }

  public StatusSignal<Integer> getVersion() {
    return cancoder.getVersion();
  }

  public StatusSignal<Integer> getVersion(boolean refresh) {
    return cancoder.getVersion(refresh);
  }

  public StatusSignal<Integer> getFaultField() {
    return cancoder.getFaultField();
  }

  public StatusSignal<Integer> getFaultField(boolean refresh) {
    return cancoder.getFaultField(refresh);
  }

  public StatusSignal<Integer> getStickyFaultField() {
    return cancoder.getStickyFaultField();
  }

  public StatusSignal<Integer> getStickyFaultField(boolean refresh) {
    return cancoder.getStickyFaultField(refresh);
  }

  public StatusSignal<AngularVelocity> getVelocity() {
    return cancoder.getVelocity();
  }

  public StatusSignal<AngularVelocity> getVelocity(boolean refresh) {
    return cancoder.getVelocity(refresh);
  }

  public StatusSignal<Angle> getPosition() {
    return cancoder.getPosition();
  }

  public StatusSignal<Angle> getPosition(boolean refresh) {
    return cancoder.getPosition(refresh);
  }

  public StatusSignal<Angle> getAbsolutePosition() {
    return cancoder.getAbsolutePosition();
  }

  public StatusSignal<Angle> getAbsolutePosition(boolean refresh) {
    return cancoder.getAbsolutePosition(refresh);
  }

  public StatusSignal<AngularVelocity> getUnfilteredVelocity() {
    return cancoder.getUnfilteredVelocity();
  }

  public StatusSignal<AngularVelocity> getUnfilteredVelocity(boolean refresh) {
    return cancoder.getUnfilteredVelocity(refresh);
  }

  public StatusSignal<Angle> getPositionSinceBoot() {
    return cancoder.getPositionSinceBoot();
  }

  public StatusSignal<Angle> getPositionSinceBoot(boolean refresh) {
    return cancoder.getPositionSinceBoot(refresh);
  }

  public StatusSignal<Voltage> getSupplyVoltage() {
    return cancoder.getSupplyVoltage();
  }

  public StatusSignal<Voltage> getSupplyVoltage(boolean refresh) {
    return cancoder.getSupplyVoltage(refresh);
  }

  public StatusSignal<MagnetHealthValue> getMagnetHealth() {
    return cancoder.getMagnetHealth();
  }

  public StatusSignal<MagnetHealthValue> getMagnetHealth(boolean refresh) {
    return cancoder.getMagnetHealth(refresh);
  }

  public StatusSignal<Boolean> getIsProLicensed() {
    return cancoder.getIsProLicensed();
  }

  public StatusSignal<Boolean> getIsProLicensed(boolean refresh) {
    return cancoder.getIsProLicensed(refresh);
  }

  public StatusSignal<Boolean> getFault_Hardware() {
    return cancoder.getFault_Hardware();
  }

  public StatusSignal<Boolean> getFault_Hardware(boolean refresh) {
    return cancoder.getFault_Hardware(refresh);
  }

  public StatusSignal<Boolean> getStickyFault_Hardware() {
    return cancoder.getStickyFault_Hardware();
  }

  public StatusSignal<Boolean> getStickyFault_Hardware(boolean refresh) {
    return cancoder.getStickyFault_Hardware(refresh);
  }

  public StatusSignal<Boolean> getFault_Undervoltage() {
    return cancoder.getFault_Undervoltage();
  }

  public StatusSignal<Boolean> getFault_Undervoltage(boolean refresh) {
    return cancoder.getFault_Undervoltage(refresh);
  }

  public StatusSignal<Boolean> getStickyFault_Undervoltage() {
    return cancoder.getStickyFault_Undervoltage();
  }

  public StatusSignal<Boolean> getStickyFault_Undervoltage(boolean refresh) {
    return cancoder.getStickyFault_Undervoltage(refresh);
  }

  public StatusSignal<Boolean> getFault_BootDuringEnable() {
    return cancoder.getFault_BootDuringEnable();
  }

  public StatusSignal<Boolean> getFault_BootDuringEnable(boolean refresh) {
    return cancoder.getFault_BootDuringEnable(refresh);
  }

  public StatusSignal<Boolean> getStickyFault_BootDuringEnable() {
    return cancoder.getStickyFault_BootDuringEnable();
  }

  public StatusSignal<Boolean> getStickyFault_BootDuringEnable(boolean refresh) {
    return cancoder.getStickyFault_BootDuringEnable(refresh);
  }

  public StatusSignal<Boolean> getFault_UnlicensedFeatureInUse() {
    return cancoder.getFault_UnlicensedFeatureInUse();
  }

  public StatusSignal<Boolean> getFault_UnlicensedFeatureInUse(boolean refresh) {
    return cancoder.getFault_UnlicensedFeatureInUse(refresh);
  }

  public StatusSignal<Boolean> getStickyFault_UnlicensedFeatureInUse() {
    return cancoder.getStickyFault_UnlicensedFeatureInUse();
  }

  public StatusSignal<Boolean> getStickyFault_UnlicensedFeatureInUse(boolean refresh) {
    return cancoder.getStickyFault_UnlicensedFeatureInUse(refresh);
  }

  public StatusSignal<Boolean> getFault_BadMagnet() {
    return cancoder.getFault_BadMagnet();
  }

  public StatusSignal<Boolean> getFault_BadMagnet(boolean refresh) {
    return cancoder.getFault_BadMagnet(refresh);
  }

  public StatusSignal<Boolean> getStickyFault_BadMagnet() {
    return cancoder.getStickyFault_BadMagnet();
  }

  public StatusSignal<Boolean> getStickyFault_BadMagnet(boolean refresh) {
    return cancoder.getStickyFault_BadMagnet(refresh);
  }

  public StatusCode setControl(ControlRequest request) {
    return cancoder.setControl(request);
  }

  public StatusCode setPosition(double newValue) {
    return cancoder.setPosition(newValue);
  }

  public StatusCode setPosition(double newValue, double timeoutSeconds) {
    return cancoder.setPosition(newValue, timeoutSeconds);
  }

  public StatusCode setPosition(Angle newValue) {
    return cancoder.setPosition(newValue);
  }

  public StatusCode setPosition(Angle newValue, double timeoutSeconds) {
    return cancoder.setPosition(newValue, timeoutSeconds);
  }

  public StatusCode clearStickyFaults() {
    return cancoder.clearStickyFaults();
  }

  public StatusCode clearStickyFaults(double timeoutSeconds) {
    return cancoder.clearStickyFaults(timeoutSeconds);
  }

  public StatusCode clearStickyFault_Hardware() {
    return cancoder.clearStickyFault_Hardware();
  }

  public StatusCode clearStickyFault_Hardware(double timeoutSeconds) {
    return cancoder.clearStickyFault_Hardware(timeoutSeconds);
  }

  public StatusCode clearStickyFault_Undervoltage() {
    return cancoder.clearStickyFault_Undervoltage();
  }

  public StatusCode clearStickyFault_Undervoltage(double timeoutSeconds) {
    return cancoder.clearStickyFault_Undervoltage(timeoutSeconds);
  }

  public StatusCode clearStickyFault_BootDuringEnable() {
    return cancoder.clearStickyFault_BootDuringEnable();
  }

  public StatusCode clearStickyFault_BootDuringEnable(double timeoutSeconds) {
    return cancoder.clearStickyFault_BootDuringEnable(timeoutSeconds);
  }

  public StatusCode clearStickyFault_UnlicensedFeatureInUse() {
    return cancoder.clearStickyFault_UnlicensedFeatureInUse();
  }

  public StatusCode clearStickyFault_UnlicensedFeatureInUse(double timeoutSeconds) {
    return cancoder.clearStickyFault_UnlicensedFeatureInUse(timeoutSeconds);
  }

  public StatusCode clearStickyFault_BadMagnet() {
    return cancoder.clearStickyFault_BadMagnet();
  }

  public StatusCode clearStickyFault_BadMagnet(double timeoutSeconds) {
    return cancoder.clearStickyFault_BadMagnet(timeoutSeconds);
  }

  public RSCANCoder(int deviceId, String canbus) {
    cancoder = new CANcoder(deviceId, canbus);
  }

  public void close() {
    cancoder.close();
  }

  public void initSendable(SendableBuilder builder) {
    cancoder.initSendable(builder);
  }

  public int getDeviceID() {
    return cancoder.getDeviceID();
  }

  public String getNetwork() {
    return cancoder.getNetwork();
  }

  public long getDeviceHash() {
    return cancoder.getDeviceHash();
  }

  public ControlRequest getAppliedControl() {
    return cancoder.getAppliedControl();
  }

  public boolean hasResetOccurred() {
    return cancoder.hasResetOccurred();
  }

  public BooleanSupplier getResetOccurredChecker() {
    return cancoder.getResetOccurredChecker();
  }

  public boolean isConnected() {
    return cancoder.isConnected();
  }

  public boolean isConnected(double maxLatencySeconds) {
    return cancoder.isConnected(maxLatencySeconds);
  }

  public StatusCode optimizeBusUtilization() {
    return cancoder.optimizeBusUtilization();
  }

  public StatusCode optimizeBusUtilization(double optimizedFreqHz, double timeoutSeconds) {
    return cancoder.optimizeBusUtilization(optimizedFreqHz, timeoutSeconds);
  }

  public StatusCode resetSignalFrequencies() {
    return cancoder.resetSignalFrequencies();
  }

  public StatusCode resetSignalFrequencies(double timeoutSeconds) {
    return cancoder.resetSignalFrequencies(timeoutSeconds);
  }
}
