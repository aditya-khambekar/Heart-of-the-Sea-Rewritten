package org.team4639.lib.control;

import edu.wpi.first.hal.util.BoundaryException;
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.util.sendable.Sendable;
import edu.wpi.first.util.sendable.SendableBuilder;
import edu.wpi.first.wpilibj.Timer;

/**
 * This class implements a PID Control Loop.
 *
 * <p>Does all computation synchronously (i.e. the calculate() function must be called by the user
 * from their own thread)
 */
public class PIDFController implements Sendable {
  private double m_P; // factor for "proportional" control
  private double m_I; // factor for "integral" control
  private double m_D; // factor for "derivative" control
  private double m_F; // factor for feed forward gain
  private double m_maximumOutput = 1.0; // |maximum output|
  private double m_minimumOutput = -1.0; // |minimum output|
  private double m_maximumInput = 0.0; // maximum input - limit setpoint to this
  private double m_minimumInput = 0.0; // minimum input - limit setpoint to this
  private boolean m_continuous = false; // do the endpoints wrap around? eg. absolute encoder
  private double m_prevError = 0.0; // the prior sensor input (used to compute velocity)
  private double m_totalError = 0.0; // the sum of the errors for use in the integral calc
  private double m_setpoint = 0.0;
  private double m_error = 0.0;
  private double m_result = 0.0;
  private double m_last_input = Double.NaN;
  private double m_deadband =
      0.0; // If the absolute error is less than deadband then treat error for the proportional term
  // as 0
  private double m_last_timestamp = Timer.getFPGATimestamp();

  public PIDFController() {}

  /**
   * Allocate a PID object with the given constants for P, I, D
   *
   * @param Kp the proportional coefficient
   * @param Ki the integral coefficient
   * @param Kd the derivative coefficient
   */
  public PIDFController(double Kp, double Ki, double Kd) {
    setPIDF(Kp, Ki, Kd, 0);
  }

  /**
   * Allocate a PID object with the given constants for P, I, D
   *
   * @param Kp the proportional coefficient
   * @param Ki the integral coefficient
   * @param Kd the derivative coefficient
   * @param Kf the feed forward gain coefficient
   */
  public PIDFController(double Kp, double Ki, double Kd, double Kf) {
    setPIDF(Kp, Ki, Kd, Kf);
  }

  public double calculate(double input) {
    double timestamp = Timer.getFPGATimestamp();
    double dt = timestamp - m_last_timestamp;
    m_last_timestamp = timestamp;

    return calculate(input, dt);
  }

  /**
   * Read the input, calculate the output accordingly, and write to the output. This should be
   * called at a constant rate by the user (ex. in a timed thread)
   *
   * @param input the input
   * @param dt time passed since previous call to calculate
   */
  public double calculate(double input, double dt) {
    if (dt < 1E-6) {
      dt = 1E-6;
    }

    m_last_input = input;
    m_error = m_setpoint - input;
    if (m_continuous) {
      if (Math.abs(m_error) > (m_maximumInput - m_minimumInput) / 2) {
        if (m_error > 0) {
          m_error = m_error - m_maximumInput + m_minimumInput;
        } else {
          m_error = m_error + m_maximumInput - m_minimumInput;
        }
      }
    }

    if (MathUtil.isNear(m_error * m_P, m_minimumOutput, m_maximumOutput)) {
      m_totalError += m_error * dt;
    } else {
      m_totalError = 0;
    }

    // Don't blow away m_error so as to not break derivative
    double proportionalError = Math.abs(m_error) < m_deadband ? 0 : m_error;

    m_result =
        (m_P * proportionalError
            + m_I * m_totalError
            + m_D * (m_error - m_prevError) / dt
            + m_F * m_setpoint);
    m_prevError = m_error;

    return MathUtil.clamp(m_result, m_minimumOutput, m_maximumOutput);
  }

  /**
   * Set the PID controller gain parameters. Set the proportional, integral, and differential
   * coefficients.
   *
   * @param p Proportional coefficient
   * @param i Integral coefficient
   * @param d Differential coefficient
   */
  public void setPID(double p, double i, double d) {
    m_P = p;
    m_I = i;
    m_D = d;
  }

  /**
   * Set the PID controller gain parameters. Set the proportional, integral, and differential
   * coefficients.
   *
   * @param p Proportional coefficient
   * @param i Integral coefficient
   * @param d Differential coefficient
   * @param f Feed forward coefficient
   */
  public void setPIDF(double p, double i, double d, double f) {
    setPID(p, i, d);
    m_F = f;
  }

  /**
   * Get the Proportional coefficient
   *
   * @return proportional coefficient
   */
  public double getP() {
    return m_P;
  }

  /**
   * Get the Integral coefficient
   *
   * @return integral coefficient
   */
  public double getI() {
    return m_I;
  }

  /**
   * Get the Differential coefficient
   *
   * @return differential coefficient
   */
  public double getD() {
    return m_D;
  }

  /**
   * Get the Feed forward coefficient
   *
   * @return feed forward coefficient
   */
  public double getF() {
    return m_F;
  }

  /**
   * Return the current PID result This is always centered on zero and constrained the the max and
   * min outs
   *
   * @return the latest calculated output
   */
  public double get() {
    return m_result;
  }

  /**
   * Set the PID controller to consider the input to be continuous, Rather then using the max and
   * min in as constraints, it considers them to be the same point and automatically calculates the
   * shortest route to the setpoint.
   *
   * @param continuous Set to true turns on continuous, false turns off continuous
   */
  public void setContinuous(boolean continuous) {
    m_continuous = continuous;
  }

  public void setDeadband(double deadband) {
    m_deadband = deadband;
  }

  /**
   * Set the PID controller to consider the input to be continuous, Rather then using the max and
   * min in as constraints, it considers them to be the same point and automatically calculates the
   * shortest route to the setpoint.
   */
  public void setContinuous() {
    this.setContinuous(true);
  }

  /**
   * Sets the maximum and minimum values expected from the input.
   *
   * @param minimumInput the minimum value expected from the input
   * @param maximumInput the maximum value expected from the output
   */
  public void setInputRange(double minimumInput, double maximumInput) {
    if (minimumInput > maximumInput) {
      throw new BoundaryException("Lower bound is greater than upper bound");
    }
    m_minimumInput = minimumInput;
    m_maximumInput = maximumInput;
    setSetpoint(m_setpoint);
  }

  /**
   * Sets the minimum and maximum values to write.
   *
   * @param minimumOutput the minimum value to write to the output
   * @param maximumOutput the maximum value to write to the output
   */
  public void setOutputRange(double minimumOutput, double maximumOutput) {
    if (minimumOutput > maximumOutput) {
      throw new BoundaryException("Lower bound is greater than upper bound");
    }
    m_minimumOutput = minimumOutput;
    m_maximumOutput = maximumOutput;
  }

  public void setMaxAbsoluteOutput(double maxAbsoluteOutput) {
    setOutputRange(-maxAbsoluteOutput, maxAbsoluteOutput);
  }

  /**
   * Set the setpoint for the PID controller
   *
   * @param setpoint the desired setpoint
   */
  public void setSetpoint(double setpoint) {
    if (m_maximumInput > m_minimumInput) {
      if (setpoint > m_maximumInput) {
        m_setpoint = m_maximumInput;
      } else if (setpoint < m_minimumInput) {
        m_setpoint = m_minimumInput;
      } else {
        m_setpoint = setpoint;
      }
    } else {
      m_setpoint = setpoint;
    }
  }

  /**
   * Returns the current setpoint of the PID controller
   *
   * @return the current setpoint
   */
  public double getSetpoint() {
    return m_setpoint;
  }

  /**
   * Returns the current difference of the input from the setpoint
   *
   * @return the current error
   */
  public double getError() {
    return m_error;
  }

  /**
   * Return true if the error is within the tolerance
   *
   * @return true if the error is less than the tolerance
   */
  public boolean onTarget(double tolerance) {
    return !Double.isNaN(m_last_input) && Math.abs(m_last_input - m_setpoint) < tolerance;
  }

  /** Reset all internal terms. */
  public void reset() {
    m_last_input = Double.NaN;
    m_prevError = 0;
    m_totalError = 0;
    m_result = 0;
    m_setpoint = 0;
  }

  public void resetIntegrator() {
    m_totalError = 0;
  }

  public String getState() {
    String lState = "";

    lState += "Kp: " + m_P + "\n";
    lState += "Ki: " + m_I + "\n";
    lState += "Kd: " + m_D + "\n";

    return lState;
  }

  public String getType() {
    return "PIDController";
  }

  public void setP(double p) {
    m_P = p;
  }

  public void setI(double i) {
    m_I = i;
  }

  public void setD(double d) {
    m_D = d;
  }

  public void setF(double f) {
    m_F = f;
  }

  @Override
  public void initSendable(SendableBuilder builder) {
    builder.setSmartDashboardType("PIDF Controller");
    builder.addDoubleProperty("kP", this::getP, this::setP);
    builder.addDoubleProperty("kI", this::getI, this::setI);
    builder.addDoubleProperty("kD", this::getD, this::setD);
    builder.addDoubleProperty("kF", this::getF, this::setF);
  }
}
