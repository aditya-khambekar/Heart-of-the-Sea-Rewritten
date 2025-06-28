package org.team4639.robot.subsystems;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.wpilibj.smartdashboard.Mechanism2d;
import edu.wpi.first.wpilibj.smartdashboard.MechanismLigament2d;
import edu.wpi.first.wpilibj.smartdashboard.MechanismRoot2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import java.util.List;

import org.team4639.robot.constants.Controls;
import org.team4639.robot.constants.FieldConstants;
import org.team4639.robot.robot.Subsystems;

public class DashboardOutputs extends SubsystemBase {
  private final double ROOT3 = Math.sqrt(3);
  private final Mechanism2d reefView = new Mechanism2d(10, 10);

  private final MechanismRoot2d ABRoot = reefView.getRoot("RootAB", 7.5, 0.67);
  private final MechanismLigament2d ABLigament =
      ABRoot.append(new MechanismLigament2d("LigamentAB", 5, 180));

  private final MechanismLigament2d KLLigament =
      ABLigament.append(new MechanismLigament2d("LigamentKL", 5, -60));

  private final MechanismLigament2d IJLigament =
      KLLigament.append(new MechanismLigament2d("LigamentIJ", 5, -60));

  private final MechanismLigament2d GHLigament =
      IJLigament.append(new MechanismLigament2d("LigamentGH", 5, -60));

  private final MechanismLigament2d EFLigament =
      GHLigament.append(new MechanismLigament2d("LigamentEF", 5, -60));

  private final MechanismLigament2d CDLigament =
      EFLigament.append(new MechanismLigament2d("LigamentCD", 5, -60));

  public DashboardOutputs() {
    ABLigament.setLineWeight(2);
    CDLigament.setLineWeight(2);
    EFLigament.setLineWeight(2);
    GHLigament.setLineWeight(2);
    IJLigament.setLineWeight(2);
    KLLigament.setLineWeight(2);

    SmartDashboard.putString("Reef Level", "NONE");
  }

  public void periodic() {
    Pose2d drivePose = Subsystems.drive.getPose();

    Pose2d nearestReefPose =
        drivePose.nearest(
            List.of(
                FieldConstants.TargetPositions.REEF_AB.getPose(),
                FieldConstants.TargetPositions.REEF_CD.getPose(),
                FieldConstants.TargetPositions.REEF_EF.getPose(),
                FieldConstants.TargetPositions.REEF_GH.getPose(),
                FieldConstants.TargetPositions.REEF_IJ.getPose(),
                FieldConstants.TargetPositions.REEF_KL.getPose()));

    if (nearestReefPose == FieldConstants.TargetPositions.REEF_AB.getPose()) {
      ABLigament.setLineWeight(15);
      CDLigament.setLineWeight(2);
      EFLigament.setLineWeight(2);
      GHLigament.setLineWeight(2);
      IJLigament.setLineWeight(2);
      KLLigament.setLineWeight(2);
    } else if (nearestReefPose == FieldConstants.TargetPositions.REEF_CD.getPose()) {
      ABLigament.setLineWeight(2);
      CDLigament.setLineWeight(15);
      EFLigament.setLineWeight(2);
      GHLigament.setLineWeight(2);
      IJLigament.setLineWeight(2);
      KLLigament.setLineWeight(2);
    } else if (nearestReefPose == FieldConstants.TargetPositions.REEF_EF.getPose()) {
      ABLigament.setLineWeight(2);
      CDLigament.setLineWeight(2);
      EFLigament.setLineWeight(15);
      GHLigament.setLineWeight(2);
      IJLigament.setLineWeight(2);
      KLLigament.setLineWeight(2);
    } else if (nearestReefPose == FieldConstants.TargetPositions.REEF_GH.getPose()) {
      ABLigament.setLineWeight(2);
      CDLigament.setLineWeight(2);
      EFLigament.setLineWeight(2);
      GHLigament.setLineWeight(15);
      IJLigament.setLineWeight(2);
      KLLigament.setLineWeight(2);
    } else if (nearestReefPose == FieldConstants.TargetPositions.REEF_IJ.getPose()) {
      ABLigament.setLineWeight(2);
      CDLigament.setLineWeight(2);
      EFLigament.setLineWeight(2);
      GHLigament.setLineWeight(2);
      IJLigament.setLineWeight(15);
      KLLigament.setLineWeight(2);
    } else if (nearestReefPose == FieldConstants.TargetPositions.REEF_KL.getPose()) {
      ABLigament.setLineWeight(2);
      CDLigament.setLineWeight(2);
      EFLigament.setLineWeight(2);
      GHLigament.setLineWeight(2);
      IJLigament.setLineWeight(2);
      KLLigament.setLineWeight(15);
    }

    SmartDashboard.putData("ReefView", reefView);
  }

  public Command displayUpcomingReefLevel(){
    return run(() -> {
      var recommended = Subsystems.reefTracker.getRecommendedLevel();
      var level = "NONE";
      if (Controls.L4Coral.getAsBoolean()) level = "L4";
      else if (Controls.L3Coral.getAsBoolean()) level = "L3";
      else if (Controls.L2Coral.getAsBoolean()) level = "L2";
      else if (Controls.L1Coral.getAsBoolean()) level = "L1";
      else if (recommended != 0) level = "L" + recommended;
      SmartDashboard.putString("Reef Level", level);
    }).finallyDo(() -> SmartDashboard.putString("Reef Level", "NONE"));
  }

  public int upcomingReefLevel(){
    var recommended = Subsystems.reefTracker.getRecommendedLevel();
    if (Controls.L4Coral.getAsBoolean()) return 4;
    else if (Controls.L3Coral.getAsBoolean()) return 3;
    else if (Controls.L2Coral.getAsBoolean()) return 2;
    else if (Controls.L1Coral.getAsBoolean()) return 1;
    else if (recommended != 0) return recommended;
    return 0;
  }
}
