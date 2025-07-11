package org.team4639.lib.oi;

import edu.wpi.first.wpilibj.GenericHID;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.event.EventLoop;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.Trigger;

/**
 * An instance of a controller that provides utils for applying deadzone to the controller axes and
 * some other things
 */
public class RSXboxController extends CommandXboxController {
  private final double DEFAULT_DEADZONE = 0.08;

  /**
   * Construct an instance of a controller.
   *
   * @param port The port index on the Driver Station that the controller is plugged into.
   */
  public RSXboxController(int port) {
    super(port);
  }

  /**
   * Get the underlying GenericHID object.
   *
   * @return the wrapped GenericHID object
   */
  @Override
  public XboxController getHID() {
    return super.getHID();
  }

  /**
   * Constructs an event instance around this button's digital signal.
   *
   * @param button the button index
   * @return an event instance representing the button's digital signal attached to the {@link
   *     CommandScheduler#getDefaultButtonLoop()} default scheduler button loop.
   * @see #button(int, EventLoop)
   */
  @Override
  public RSTrigger button(int button) {
    return RSTrigger.of(super.button(button));
  }

  /**
   * Constructs an event instance around this button's digital signal.
   *
   * @param button the button index
   * @param loop the event loop instance to attach the event to.
   * @return an event instance representing the button's digital signal attached to the given loop.
   */
  @Override
  public RSTrigger button(int button, EventLoop loop) {
    return RSTrigger.of(super.button(button, loop));
  }

  /**
   * Constructs a Trigger instance based around this angle of the default (index 0) POV on the HID,
   * attached to {@link CommandScheduler#getDefaultButtonLoop() the default command scheduler button
   * loop}.
   *
   * <p>The POV angles start at 0 in the up direction, and increase clockwise (e.g. right is 90,
   * upper-left is 315).
   *
   * @param angle POV angle in degrees, or -1 for the center / not pressed.
   * @return a Trigger instance based around this angle of a POV on the HID.
   */
  @Override
  public RSTrigger pov(int angle) {
    return RSTrigger.of(super.pov(angle));
  }

  /**
   * Constructs a Trigger instance based around this angle of a POV on the HID.
   *
   * <p>The POV angles start at 0 in the up direction, and increase clockwise (e.g. right is 90,
   * upper-left is 315).
   *
   * @param pov index of the POV to read (starting at 0). Defaults to 0.
   * @param angle POV angle in degrees, or -1 for the center / not pressed.
   * @param loop the event loop instance to attach the event to. Defaults to {@link
   *     CommandScheduler#getDefaultButtonLoop() the default command scheduler button loop}.
   * @return a Trigger instance based around this angle of a POV on the HID.
   */
  @Override
  public RSTrigger pov(int pov, int angle, EventLoop loop) {
    return RSTrigger.of(super.pov(pov, angle, loop));
  }

  /**
   * Constructs a Trigger instance based around the 0 degree angle (up) of the default (index 0) POV
   * on the HID, attached to {@link CommandScheduler#getDefaultButtonLoop() the default command
   * scheduler button loop}.
   *
   * @return a Trigger instance based around the 0 degree angle of a POV on the HID.
   */
  @Override
  public RSTrigger povUp() {
    return RSTrigger.of(super.povUp());
  }

  /**
   * Constructs a Trigger instance based around the 45 degree angle (right up) of the default (index
   * 0) POV on the HID, attached to {@link CommandScheduler#getDefaultButtonLoop() the default
   * command scheduler button loop}.
   *
   * @return a Trigger instance based around the 45 degree angle of a POV on the HID.
   */
  @Override
  public RSTrigger povUpRight() {
    return RSTrigger.of(super.povUpRight());
  }

  /**
   * Constructs a Trigger instance based around the 90 degree angle (right) of the default (index 0)
   * POV on the HID, attached to {@link CommandScheduler#getDefaultButtonLoop() the default command
   * scheduler button loop}.
   *
   * @return a Trigger instance based around the 90 degree angle of a POV on the HID.
   */
  @Override
  public RSTrigger povRight() {
    return RSTrigger.of(super.povRight());
  }

  /**
   * Constructs a Trigger instance based around the 135 degree angle (right down) of the default
   * (index 0) POV on the HID, attached to {@link CommandScheduler#getDefaultButtonLoop() the
   * default command scheduler button loop}.
   *
   * @return a Trigger instance based around the 135 degree angle of a POV on the HID.
   */
  @Override
  public RSTrigger povDownRight() {
    return RSTrigger.of(super.povDownRight());
  }

  /**
   * Constructs a Trigger instance based around the 180 degree angle (down) of the default (index 0)
   * POV on the HID, attached to {@link CommandScheduler#getDefaultButtonLoop() the default command
   * scheduler button loop}.
   *
   * @return a Trigger instance based around the 180 degree angle of a POV on the HID.
   */
  @Override
  public RSTrigger povDown() {
    return RSTrigger.of(super.povDown());
  }

  /**
   * Constructs a Trigger instance based around the 225 degree angle (down left) of the default
   * (index 0) POV on the HID, attached to {@link CommandScheduler#getDefaultButtonLoop() the
   * default command scheduler button loop}.
   *
   * @return a Trigger instance based around the 225 degree angle of a POV on the HID.
   */
  @Override
  public RSTrigger povDownLeft() {
    return RSTrigger.of(super.povDownLeft());
  }

  /**
   * Constructs a Trigger instance based around the 270 degree angle (left) of the default (index 0)
   * POV on the HID, attached to {@link CommandScheduler#getDefaultButtonLoop() the default command
   * scheduler button loop}.
   *
   * @return a Trigger instance based around the 270 degree angle of a POV on the HID.
   */
  @Override
  public RSTrigger povLeft() {
    return RSTrigger.of(super.povLeft());
  }

  /**
   * Constructs a Trigger instance based around the 315 degree angle (left up) of the default (index
   * 0) POV on the HID, attached to {@link CommandScheduler#getDefaultButtonLoop() the default
   * command scheduler button loop}.
   *
   * @return a Trigger instance based around the 315 degree angle of a POV on the HID.
   */
  @Override
  public RSTrigger povUpLeft() {
    return RSTrigger.of(super.povUpLeft());
  }

  /**
   * Constructs a Trigger instance based around the center (not pressed) position of the default
   * (index 0) POV on the HID, attached to {@link CommandScheduler#getDefaultButtonLoop() the
   * default command scheduler button loop}.
   *
   * @return a Trigger instance based around the center position of a POV on the HID.
   */
  @Override
  public RSTrigger povCenter() {
    return RSTrigger.of(super.povCenter());
  }

  /**
   * Constructs a Trigger instance that is true when the axis value is less than {@code threshold},
   * attached to {@link CommandScheduler#getDefaultButtonLoop() the default command scheduler button
   * loop}.
   *
   * @param axis The axis to read, starting at 0
   * @param threshold The value below which this trigger should return true.
   * @return a Trigger instance that is true when the axis value is less than the provided
   *     threshold.
   */
  @Override
  public RSTrigger axisLessThan(int axis, double threshold) {
    return RSTrigger.of(super.axisLessThan(axis, threshold));
  }

  /**
   * Constructs a Trigger instance that is true when the axis value is less than {@code threshold},
   * attached to the given loop.
   *
   * @param axis The axis to read, starting at 0
   * @param threshold The value below which this trigger should return true.
   * @param loop the event loop instance to attach the trigger to
   * @return a Trigger instance that is true when the axis value is less than the provided
   *     threshold.
   */
  @Override
  public RSTrigger axisLessThan(int axis, double threshold, EventLoop loop) {
    return RSTrigger.of(super.axisLessThan(axis, threshold, loop));
  }

  /**
   * Constructs a Trigger instance that is true when the axis value is less than {@code threshold},
   * attached to {@link CommandScheduler#getDefaultButtonLoop() the default command scheduler button
   * loop}.
   *
   * @param axis The axis to read, starting at 0
   * @param threshold The value above which this trigger should return true.
   * @return a Trigger instance that is true when the axis value is greater than the provided
   *     threshold.
   */
  @Override
  public RSTrigger axisGreaterThan(int axis, double threshold) {
    return RSTrigger.of(super.axisGreaterThan(axis, threshold));
  }

  /**
   * Constructs a Trigger instance that is true when the axis value is greater than {@code
   * threshold}, attached to the given loop.
   *
   * @param axis The axis to read, starting at 0
   * @param threshold The value above which this trigger should return true.
   * @param loop the event loop instance to attach the trigger to.
   * @return a Trigger instance that is true when the axis value is greater than the provided
   *     threshold.
   */
  @Override
  public RSTrigger axisGreaterThan(int axis, double threshold, EventLoop loop) {
    return RSTrigger.of(super.axisGreaterThan(axis, threshold, loop));
  }

  /**
   * Constructs a Trigger instance that is true when the axis magnitude value is greater than {@code
   * threshold}, attached to the given loop.
   *
   * @param axis The axis to read, starting at 0
   * @param threshold The value above which this trigger should return true.
   * @param loop the event loop instance to attach the trigger to.
   * @return a Trigger instance that is true when the axis magnitude value is greater than the
   *     provided threshold.
   */
  @Override
  public RSTrigger axisMagnitudeGreaterThan(int axis, double threshold, EventLoop loop) {
    return RSTrigger.of(super.axisMagnitudeGreaterThan(axis, threshold, loop));
  }

  /**
   * Constructs a Trigger instance that is true when the axis magnitude value is greater than {@code
   * threshold}, attached to {@link CommandScheduler#getDefaultButtonLoop() the default command
   * scheduler button loop}.
   *
   * @param axis The axis to read, starting at 0
   * @param threshold The value above which this trigger should return true.
   * @return a Trigger instance that is true when the deadbanded axis value is active (non-zero).
   */
  @Override
  public RSTrigger axisMagnitudeGreaterThan(int axis, double threshold) {
    return RSTrigger.of(super.axisMagnitudeGreaterThan(axis, threshold));
  }

  /**
   * Get the value of the axis.
   *
   * @param axis The axis to read, starting at 0.
   * @return The value of the axis.
   */
  @Override
  public double getRawAxis(int axis) {
    return super.getRawAxis(axis);
  }

  /**
   * Set the rumble output for the HID. The DS currently supports 2 rumble values, left rumble and
   * right rumble.
   *
   * @param type Which rumble value to set
   * @param value The normalized value (0 to 1) to set the rumble to
   */
  @Override
  public void setRumble(GenericHID.RumbleType type, double value) {
    super.setRumble(type, value);
  }

  /**
   * Get if the HID is connected.
   *
   * @return true if the HID is connected
   */
  @Override
  public boolean isConnected() {
    return super.isConnected();
  }

  /**
   * Constructs a Trigger instance around the A button's digital signal.
   *
   * @return a Trigger instance representing the A button's digital signal attached to the {@link
   *     CommandScheduler#getDefaultButtonLoop() default scheduler button loop}.
   * @see #a(EventLoop)
   */
  @Override
  public RSTrigger a() {
    return RSTrigger.of(super.a());
  }

  /**
   * Constructs a Trigger instance around the A button's digital signal.
   *
   * @param loop the event loop instance to attach the event to.
   * @return a Trigger instance representing the A button's digital signal attached to the given
   *     loop.
   */
  @Override
  public RSTrigger a(EventLoop loop) {
    return RSTrigger.of(super.a(loop));
  }

  /**
   * Constructs a Trigger instance around the B button's digital signal.
   *
   * @return a Trigger instance representing the B button's digital signal attached to the {@link
   *     CommandScheduler#getDefaultButtonLoop() default scheduler button loop}.
   * @see #b(EventLoop)
   */
  @Override
  public RSTrigger b() {
    return RSTrigger.of(super.b());
  }

  /**
   * Constructs a Trigger instance around the B button's digital signal.
   *
   * @param loop the event loop instance to attach the event to.
   * @return a Trigger instance representing the B button's digital signal attached to the given
   *     loop.
   */
  @Override
  public RSTrigger b(EventLoop loop) {
    return RSTrigger.of(super.b(loop));
  }

  /**
   * Constructs a Trigger instance around the X button's digital signal.
   *
   * @return a Trigger instance representing the X button's digital signal attached to the {@link
   *     CommandScheduler#getDefaultButtonLoop() default scheduler button loop}.
   * @see #x(EventLoop)
   */
  @Override
  public RSTrigger x() {
    return RSTrigger.of(super.x());
  }

  /**
   * Constructs a Trigger instance around the X button's digital signal.
   *
   * @param loop the event loop instance to attach the event to.
   * @return a Trigger instance representing the X button's digital signal attached to the given
   *     loop.
   */
  @Override
  public RSTrigger x(EventLoop loop) {
    return RSTrigger.of(super.x(loop));
  }

  /**
   * Constructs a Trigger instance around the Y button's digital signal.
   *
   * @return a Trigger instance representing the Y button's digital signal attached to the {@link
   *     CommandScheduler#getDefaultButtonLoop() default scheduler button loop}.
   * @see #y(EventLoop)
   */
  @Override
  public RSTrigger y() {
    return RSTrigger.of(super.y());
  }

  /**
   * Constructs a Trigger instance around the Y button's digital signal.
   *
   * @param loop the event loop instance to attach the event to.
   * @return a Trigger instance representing the Y button's digital signal attached to the given
   *     loop.
   */
  @Override
  public RSTrigger y(EventLoop loop) {
    return RSTrigger.of(super.y(loop));
  }

  /**
   * Constructs a Trigger instance around the left bumper button's digital signal.
   *
   * @return a Trigger instance representing the left bumper button's digital signal attached to the
   *     {@link CommandScheduler#getDefaultButtonLoop() default scheduler button loop}.
   * @see #leftBumper(EventLoop)
   */
  @Override
  public RSTrigger leftBumper() {
    return RSTrigger.of(super.leftBumper());
  }

  /**
   * Constructs a Trigger instance around the left bumper button's digital signal.
   *
   * @param loop the event loop instance to attach the event to.
   * @return a Trigger instance representing the left bumper button's digital signal attached to the
   *     given loop.
   */
  @Override
  public RSTrigger leftBumper(EventLoop loop) {
    return RSTrigger.of(super.leftBumper(loop));
  }

  /**
   * Constructs a Trigger instance around the right bumper button's digital signal.
   *
   * @return a Trigger instance representing the right bumper button's digital signal attached to
   *     the {@link CommandScheduler#getDefaultButtonLoop() default scheduler button loop}.
   * @see #rightBumper(EventLoop)
   */
  @Override
  public RSTrigger rightBumper() {
    return RSTrigger.of(super.rightBumper());
  }

  /**
   * Constructs a Trigger instance around the right bumper button's digital signal.
   *
   * @param loop the event loop instance to attach the event to.
   * @return a Trigger instance representing the right bumper button's digital signal attached to
   *     the given loop.
   */
  @Override
  public RSTrigger rightBumper(EventLoop loop) {
    return RSTrigger.of(super.rightBumper(loop));
  }

  /**
   * Constructs a Trigger instance around the back button's digital signal.
   *
   * @return a Trigger instance representing the back button's digital signal attached to the {@link
   *     CommandScheduler#getDefaultButtonLoop() default scheduler button loop}.
   * @see #back(EventLoop)
   */
  @Override
  public RSTrigger back() {
    return RSTrigger.of(super.back());
  }

  /**
   * Constructs a Trigger instance around the back button's digital signal.
   *
   * @param loop the event loop instance to attach the event to.
   * @return a Trigger instance representing the back button's digital signal attached to the given
   *     loop.
   */
  @Override
  public RSTrigger back(EventLoop loop) {
    return RSTrigger.of(super.back(loop));
  }

  /**
   * Constructs a Trigger instance around the start button's digital signal.
   *
   * @return a Trigger instance representing the start button's digital signal attached to the
   *     {@link CommandScheduler#getDefaultButtonLoop() default scheduler button loop}.
   * @see #start(EventLoop)
   */
  @Override
  public RSTrigger start() {
    return RSTrigger.of(super.start());
  }

  /**
   * Constructs a Trigger instance around the start button's digital signal.
   *
   * @param loop the event loop instance to attach the event to.
   * @return a Trigger instance representing the start button's digital signal attached to the given
   *     loop.
   */
  @Override
  public RSTrigger start(EventLoop loop) {
    return RSTrigger.of(super.start(loop));
  }

  /**
   * Constructs a Trigger instance around the left stick button's digital signal.
   *
   * @return a Trigger instance representing the left stick button's digital signal attached to the
   *     {@link CommandScheduler#getDefaultButtonLoop() default scheduler button loop}.
   * @see #leftStick(EventLoop)
   */
  @Override
  public RSTrigger leftStick() {
    return RSTrigger.of(super.leftStick());
  }

  /**
   * Constructs a Trigger instance around the left stick button's digital signal.
   *
   * @param loop the event loop instance to attach the event to.
   * @return a Trigger instance representing the left stick button's digital signal attached to the
   *     given loop.
   */
  @Override
  public RSTrigger leftStick(EventLoop loop) {
    return RSTrigger.of(super.leftStick(loop));
  }

  /**
   * Constructs a Trigger instance around the right stick button's digital signal.
   *
   * @return a Trigger instance representing the right stick button's digital signal attached to the
   *     {@link CommandScheduler#getDefaultButtonLoop() default scheduler button loop}.
   * @see #rightStick(EventLoop)
   */
  @Override
  public RSTrigger rightStick() {
    return RSTrigger.of(super.rightStick());
  }

  /**
   * Constructs a Trigger instance around the right stick button's digital signal.
   *
   * @param loop the event loop instance to attach the event to.
   * @return a Trigger instance representing the right stick button's digital signal attached to the
   *     given loop.
   */
  @Override
  public RSTrigger rightStick(EventLoop loop) {
    return RSTrigger.of(super.rightStick(loop));
  }

  /**
   * Constructs a Trigger instance around the axis value of the left trigger. The returned trigger
   * will be true when the axis value is greater than {@code threshold}.
   *
   * @param threshold the minimum axis value for the returned {@link Trigger} to be true. This value
   *     should be in the range [0, 1] where 0 is the unpressed state of the axis.
   * @param loop the event loop instance to attach the Trigger to.
   * @return a Trigger instance that is true when the left trigger's axis exceeds the provided
   *     threshold, attached to the given event loop
   */
  @Override
  public RSTrigger leftTrigger(double threshold, EventLoop loop) {
    return RSTrigger.of(super.leftTrigger(threshold, loop));
  }

  /**
   * Constructs a Trigger instance around the axis value of the left trigger. The returned trigger
   * will be true when the axis value is greater than {@code threshold}.
   *
   * @param threshold the minimum axis value for the returned {@link Trigger} to be true. This value
   *     should be in the range [0, 1] where 0 is the unpressed state of the axis.
   * @return a Trigger instance that is true when the left trigger's axis exceeds the provided
   *     threshold, attached to the {@link CommandScheduler#getDefaultButtonLoop() default scheduler
   *     button loop}.
   */
  @Override
  public RSTrigger leftTrigger(double threshold) {
    return RSTrigger.of(super.leftTrigger(threshold));
  }

  /**
   * Constructs a Trigger instance around the axis value of the left trigger. The returned trigger
   * will be true when the axis value is greater than 0.5.
   *
   * @return a Trigger instance that is true when the left trigger's axis exceeds 0.5, attached to
   *     the {@link CommandScheduler#getDefaultButtonLoop() default scheduler button loop}.
   */
  @Override
  public RSTrigger leftTrigger() {
    return RSTrigger.of(super.leftTrigger());
  }

  /**
   * Constructs a Trigger instance around the axis value of the right trigger. The returned trigger
   * will be true when the axis value is greater than {@code threshold}.
   *
   * @param threshold the minimum axis value for the returned {@link Trigger} to be true. This value
   *     should be in the range [0, 1] where 0 is the unpressed state of the axis.
   * @param loop the event loop instance to attach the Trigger to.
   * @return a Trigger instance that is true when the right trigger's axis exceeds the provided
   *     threshold, attached to the given event loop
   */
  @Override
  public RSTrigger rightTrigger(double threshold, EventLoop loop) {
    return RSTrigger.of(super.rightTrigger(threshold, loop));
  }

  /**
   * Constructs a Trigger instance around the axis value of the right trigger. The returned trigger
   * will be true when the axis value is greater than {@code threshold}.
   *
   * @param threshold the minimum axis value for the returned {@link Trigger} to be true. This value
   *     should be in the range [0, 1] where 0 is the unpressed state of the axis.
   * @return a Trigger instance that is true when the right trigger's axis exceeds the provided
   *     threshold, attached to the {@link CommandScheduler#getDefaultButtonLoop() default scheduler
   *     button loop}.
   */
  @Override
  public RSTrigger rightTrigger(double threshold) {
    return RSTrigger.of(super.rightTrigger(threshold));
  }

  /**
   * Constructs a Trigger instance around the axis value of the right trigger. The returned trigger
   * will be true when the axis value is greater than 0.5.
   *
   * @return a Trigger instance that is true when the right trigger's axis exceeds 0.5, attached to
   *     the {@link CommandScheduler#getDefaultButtonLoop() default scheduler button loop}.
   */
  @Override
  public RSTrigger rightTrigger() {
    return RSTrigger.of(super.rightTrigger());
  }

  /**
   * Get the X axis value of left side of the controller. Right is positive.
   *
   * @return The axis value.
   */
  @Override
  public double getLeftX() {
    return applyDeadzone(super.getLeftX());
  }

  /**
   * Get the X axis value of right side of the controller. Right is positive.
   *
   * @return The axis value.
   */
  @Override
  public double getRightX() {
    return applyDeadzone(super.getRightX());
  }

  /**
   * Get the Y axis value of left side of the controller. Back is positive.
   *
   * @return The axis value.
   */
  @Override
  public double getLeftY() {
    return applyDeadzone(super.getLeftY());
  }

  /**
   * Get the Y axis value of right side of the controller. Back is positive.
   *
   * @return The axis value.
   */
  @Override
  public double getRightY() {
    return applyDeadzone(super.getRightY());
  }

  /**
   * Get the left trigger axis value of the controller. Note that this axis is bound to the range of
   * [0, 1] as opposed to the usual [-1, 1].
   *
   * @return The axis value.
   */
  @Override
  public double getLeftTriggerAxis() {
    return super.getLeftTriggerAxis();
  }

  /**
   * Get the right trigger axis value of the controller. Note that this axis is bound to the range
   * of [0, 1] as opposed to the usual [-1, 1].
   *
   * @return The axis value.
   */
  @Override
  public double getRightTriggerAxis() {
    return super.getRightTriggerAxis();
  }

  public double applyDeadzone(double value) {
    return Math.signum(value) * Math.max((Math.abs(value) - DEFAULT_DEADZONE), 0);
  }

  private RSTrigger ab = RSTrigger.of(a().and(b()));
  private RSTrigger xy = RSTrigger.of(x().and(y()));

  /**
   * Functionally equivalent to a().and(b())
   *
   * @return a trigger representing both a and b buttons being pressed.
   */
  public RSTrigger ab() {
    return ab;
  }

  /**
   * Functionally equivalent to x().and(y())
   *
   * @return a trigger representing both x and y buttons being pressed.
   */
  public RSTrigger xy() {
    return xy;
  }
}