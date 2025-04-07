package org.team4639._lib.motorcontrol.talonfx;

public final class RSTalonFXTemplate {
  public static RSTalonFX KrakenX60(int ID) {
    return new RSTalonFX(ID);
  }

  public static RSTalonFX KrakenX60(int ID, String canbus) {
    return new RSTalonFX(ID, canbus);
  }
}
