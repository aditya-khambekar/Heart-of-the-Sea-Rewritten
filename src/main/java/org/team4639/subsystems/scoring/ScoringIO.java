package org.team4639.subsystems.scoring;

import org.littletonrobotics.junction.AutoLog;

import com.revrobotics.spark.SparkMax;

public interface ScoringIO {
    
    @AutoLog
    class ScoringIOInputs {
        double currentAmps;
    }

    default void updateInputs(ScoringIOInputs inputs){}

    default void sendSparkMaxData(SparkMax spark){}
}
