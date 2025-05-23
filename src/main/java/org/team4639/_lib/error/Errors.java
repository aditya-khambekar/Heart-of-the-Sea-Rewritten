package org.team4639._lib.error;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import java.util.ArrayList;
import java.util.HashMap;
import java.util.List;
import java.util.Map;
import java.util.concurrent.atomic.AtomicBoolean;
import java.util.function.BooleanSupplier;
import org.team4639._lib.util.Util;

public class Errors {
  public static boolean hasError = false;
  private static Map<BooleanSupplier, String> errorMap = new HashMap<>();
  private static double timeAtStart = Timer.getFPGATimestamp();

  /**
   * Add an error check that will be checked at a 1hz rate.
   *
   * @param check returns true if there is no error, and false if there is an error.
   * @param errorMessage
   */
  public static void addCheck(BooleanSupplier check, String errorMessage) {
    errorMap.put(check, errorMessage);
  }

  public static void checkForErrors() {
    if (Util.inRange((Timer.getFPGATimestamp() - timeAtStart) % 1.0, -0.05, 0.05)) {
      List<String> errors = new ArrayList<>();
      AtomicBoolean _hasError = new AtomicBoolean(false);
      errorMap.keySet().stream()
          .parallel()
          .forEach(
              b -> {
                if (b.getAsBoolean()) {
                  _hasError.set(true);
                  errors.add(errorMap.get(b));
                }
              });
      hasError = _hasError.get();

      SmartDashboard.putBoolean("Error/Has Error", hasError);
      SmartDashboard.putStringArray("Error/Errors", errors.toArray(new String[0]));
    }
  }
}
