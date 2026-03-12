package frc.robot.commands;

import com.pathplanner.lib.controllers.PPHolonomicDriveController;
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.vision.Vision;

public class AutoTagRotationOverride extends Command {
  private final Vision vision;
  private final PIDController turnPid = new PIDController(0.03, 0.0, 0.001);

  public AutoTagRotationOverride(Vision vision) {
    this.vision = vision;
    turnPid.setTolerance(1.5);
  }

  private int getTargetTagId() {
    var alliance = DriverStation.getAlliance();
    if (alliance.isPresent() && alliance.get() == DriverStation.Alliance.Red) {
      return 9; // change to your red tag
    } else {
      return 26; // change to your blue tag
    }
  }

  @Override
  public void initialize() {
    PPHolonomicDriveController.overrideRotationFeedback(
        () -> {
          int desiredTagId = getTargetTagId();

          if (vision.seesTag(0, desiredTagId)) {
            double omega = turnPid.calculate(vision.getTargetXDegreesForTag(0, desiredTagId), 0.0);
            return MathUtil.clamp(omega, -2.5, 2.5);
          }

          return 0.0;
        });
  }

  @Override
  public void end(boolean interrupted) {
    PPHolonomicDriveController.clearRotationFeedbackOverride();
  }

  @Override
  public boolean isFinished() {
    return false;
  }
}
