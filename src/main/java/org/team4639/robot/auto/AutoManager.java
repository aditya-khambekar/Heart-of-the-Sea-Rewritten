package org.team4639.robot.auto;

import java.util.LinkedList;
import java.util.Objects;
import java.util.Queue;

public class AutoManager {
    private static volatile AutoManager instance;

    public static synchronized AutoManager getInstance() {
        return instance = Objects.requireNonNullElseGet(instance, AutoManager::new);
    }

    private Queue<AutoGenerator.Location> tasks = new LinkedList<AutoGenerator.Location>();

    private enum inCoralDrivetrainState{
        PATH_IN,
        ALIGN,
        STOP,
        PATH_RETRY
    }

    private enum inCoralSuperstructureState{
        INTAKE_MOTION,
        MOTION_FAIL,
        INTAKE_WAIT,
        INTAKE_FAIL
    }

    private enum outCoralDrivetrainState{
        PATH_IN,
        ALIGN,
        STOP,
        PATH_RETRY,
    }

    private enum outCoralSuperstructureState{
        DOWN,
        PREP,
        UP,
        OUTTAKE,
        OUTTAKE_FAIL
    }

    public void periodic(){

    }
}
