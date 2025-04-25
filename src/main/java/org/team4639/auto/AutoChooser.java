package org.team4639.auto;

import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj2.command.Command;
import java.util.Objects;

public class AutoChooser extends SendableChooser<Command> {
  private static volatile AutoChooser instance;

  public static synchronized AutoChooser getInstance() {
    return instance = Objects.requireNonNullElseGet(instance, AutoChooser::new);
  }

  private AutoChooser() {
    // add autos here once compiled
  }
}
