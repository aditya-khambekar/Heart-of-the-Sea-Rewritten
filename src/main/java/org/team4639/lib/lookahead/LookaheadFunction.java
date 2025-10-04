package org.team4639.lib.lookahead;

public interface LookaheadFunction<T> {
  public T predict(double time);

  public void addSample(T measurement, double time);
}
