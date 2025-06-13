package org.team4639.robot.subsystems.superstructure.wrist.io;

import com.revrobotics.spark.SparkFlex;
import com.revrobotics.spark.SparkLowLevel;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.Dimensionless;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

public class WristIOSparkFlex extends WristIO {
    SparkFlex sparkFlex;
    ProfiledPIDController wristPIDController;

    public WristIOSparkFlex(int ID) {
        sparkFlex = new SparkFlex(ID, SparkLowLevel.MotorType.kBrushless);

        wristPIDController = new ProfiledPIDController(
                40,
                0,
                0,
                new TrapezoidProfile.Constraints(
                        60, 20
                )
        );

        SmartDashboard.putData("Wrist PID Controller", wristPIDController);
    }

    @Override
    public void updateInputs(WristIOInputs inputs) {

    }

    @Override
    public void setDutyCycleOutput(Dimensionless percent) {

    }

    @Override
    public void setPosition(Angle position) {

    }
}
