package frc.robot.commands;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.drive.Drive;
import frc.robot.subsystems.vision.Vision;
import java.util.function.DoubleSupplier;

public class FaceTagWhileDriving extends Command {
  private final Drive drive;
  private final Vision vision;
  private final int desiredTagId;
  private final DoubleSupplier xSupplier;
  private final DoubleSupplier ySupplier;

  private final PIDController turnPid = new PIDController(0.03, 0.0, 0.001);

  public FaceTagWhileDriving(
      Drive drive,
      Vision vision,
      int desiredTagId,
      DoubleSupplier xSupplier,
      DoubleSupplier ySupplier) {
    this.drive = drive;
    this.vision = vision;
    this.desiredTagId = desiredTagId;
    this.xSupplier = xSupplier;
    this.ySupplier = ySupplier;

    turnPid.setTolerance(1.5);
    addRequirements(drive);
  }

  @Override
  public void execute() {
    double xSpeed = xSupplier.getAsDouble();
    double ySpeed = ySupplier.getAsDouble();
    double omega = 0.0;

    if (vision.seesTag(0, desiredTagId)) {
      omega = turnPid.calculate(vision.getTargetXDegreesForTag(0, desiredTagId), 0.0);
      omega = MathUtil.clamp(omega, -2.5, 2.5);
    }

    drive.runVelocity(
        ChassisSpeeds.fromFieldRelativeSpeeds(
            xSpeed,
            ySpeed,
            omega,
            drive.getRotation()));
  }

  @Override
  public void end(boolean interrupted) {
    drive.stop();
  }
}