package org.team4639.robot.robot;

import org.ironmaple.simulation.SimulatedArena;
import org.littletonrobotics.junction.Logger;

public class SimField {
  public static void addFieldGamePieces() {
    Logger.recordOutput(
        "FieldSimulation/Algae", SimulatedArena.getInstance().getGamePiecesArrayByType("Algae"));
    Logger.recordOutput(
        "FieldSimulation/Coral", SimulatedArena.getInstance().getGamePiecesArrayByType("Coral"));
  }
}
