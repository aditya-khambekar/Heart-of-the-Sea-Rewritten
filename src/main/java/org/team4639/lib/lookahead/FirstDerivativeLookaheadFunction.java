package org.team4639.lib.lookahead;

public class FirstDerivativeLookaheadFunction implements LookaheadFunction<Double> {
  private Double lastMeasurement;
  private Double lastTime;
  private Double secondLastMeasurement;
  private Double secondLastTime;

  public FirstDerivativeLookaheadFunction() {
    this.lastMeasurement = null;
    this.lastTime = null;
    this.secondLastMeasurement = null;
    this.secondLastTime = null;
  }

  @Override
  public Double predict(double time) {
    // If we don't have enough samples, return the last measurement or 0
    if (lastMeasurement == null) {
      return 0.0;
    }

    if (secondLastMeasurement == null || lastTime.equals(secondLastTime)) {
      // Only one sample or same timestamps, return last measurement
      return lastMeasurement;
    }

    // Calculate the first derivative (velocity)
    double dt = lastTime - secondLastTime;
    double derivative = (lastMeasurement - secondLastMeasurement) / dt;

    // Extrapolate using linear prediction: value = lastValue + derivative * timeAhead
    double timeAhead = time - lastTime;
    return lastMeasurement + derivative * timeAhead;
  }

  @Override
  public void addSample(Double measurement, double time) {
    // Shift the samples: last becomes second-last, new becomes last
    secondLastMeasurement = lastMeasurement;
    secondLastTime = lastTime;
    lastMeasurement = measurement;
    lastTime = time;
  }
}
