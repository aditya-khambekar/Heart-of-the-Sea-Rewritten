package org.team4639.lib.lookahead;

public class SecondDerivativeLookaheadFunction implements LookaheadFunction<Double> {
  private Double[] measurements;
  private Double[] times;
  private int sampleCount;

  public SecondDerivativeLookaheadFunction() {
    this.measurements = new Double[3];
    this.times = new Double[3];
    this.sampleCount = 0;
  }

  @Override
  public Double predict(double time) {
    // If we don't have enough samples, fall back to simpler predictions
    if (sampleCount == 0) {
      return 0.0;
    }

    if (sampleCount == 1) {
      // Only one sample, return it
      return measurements[0];
    }

    if (sampleCount == 2) {
      // Two samples: use first derivative (linear extrapolation)
      double dt = times[1] - times[0];
      if (dt == 0) {
        return measurements[1];
      }
      double derivative = (measurements[1] - measurements[0]) / dt;
      double timeAhead = time - times[1];
      return measurements[1] + derivative * timeAhead;
    }

    // Three samples: use second derivative (quadratic extrapolation)
    double t0 = times[0];
    double t1 = times[1];
    double t2 = times[2];
    double y0 = measurements[0];
    double y1 = measurements[1];
    double y2 = measurements[2];

    // Check for duplicate timestamps
    if (t2 == t1 || t1 == t0 || t2 == t0) {
      return measurements[2];
    }

    // Calculate first derivatives
    double v1 = (y1 - y0) / (t1 - t0); // velocity between samples 0 and 1
    double v2 = (y2 - y1) / (t2 - t1); // velocity between samples 1 and 2

    // Calculate second derivative (acceleration)
    double dt = (t2 - t0) / 2.0; // average time interval
    double acceleration = (v2 - v1) / dt;

    // Quadratic extrapolation: y = y0 + v*t + 0.5*a*t^2
    double timeAhead = time - t2;
    double prediction = y2 + v2 * timeAhead + 0.5 * acceleration * timeAhead * timeAhead;

    return prediction;
  }

  @Override
  public void addSample(Double measurement, double time) {
    if (sampleCount < 3) {
      // Still filling up the initial samples
      measurements[sampleCount] = measurement;
      times[sampleCount] = time;
      sampleCount++;
    } else {
      // Shift samples: [0] <- [1] <- [2] <- new
      measurements[0] = measurements[1];
      measurements[1] = measurements[2];
      times[0] = times[1];
      times[1] = times[2];
      measurements[2] = measurement;
      times[2] = time;
    }
  }
}
