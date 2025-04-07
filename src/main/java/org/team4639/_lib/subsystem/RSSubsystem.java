package org.team4639._lib.subsystem;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import java.util.HashSet;
import java.util.Set;

/**
 * Subsystem that integrates functionality from the other objects in this library. Make sure to call
 * {@link RSSubsystem#periodic() super.periodic()} in your subclass's periodic() method.
 */
public class RSSubsystem extends SubsystemBase {
  private final Set<Updatable> updatables;

  public RSSubsystem() {
    updatables = new HashSet<>();
  }

  @Override
  public void periodic() {
    synchronized (updatables) {
      updatables.forEach(Updatable::update);
    }
  }

  public <T extends Updatable> T addMember(T member) {
    updatables.add(member);
    return member;
  }
}
