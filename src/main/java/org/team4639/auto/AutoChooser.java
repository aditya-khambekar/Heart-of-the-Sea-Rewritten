package org.team4639.auto;

import static org.team4639.auto.AutoFactory.Locations;

import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj2.command.Command;
import java.util.Objects;

public class AutoChooser extends SendableChooser<Command> {
  private static volatile AutoChooser instance;

  public static synchronized AutoChooser getInstance() {
    return instance = Objects.requireNonNullElseGet(instance, AutoChooser::new);
  }

  private AutoChooser() {
    this.addOption("Right Side", compileRightSideAuto());
  }

  public Command compileRightSideAuto() {
    return AutoFactory.compileAuto(
        Locations.RS, Locations.E, Locations.RHP, Locations.D, Locations.RHP, Locations.C);
  }
}
