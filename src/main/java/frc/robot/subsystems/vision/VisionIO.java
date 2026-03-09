package frc.robot.subsystems.vision;

import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation2d;
import org.littletonrobotics.junction.AutoLog;

public interface VisionIO {
  @AutoLog
  public static class VisionIOInputs {
    public boolean connected = false;

    // Best target observation (kept for simple servoing)
    public TargetObservation latestTargetObservation = new TargetObservation(-1, new Rotation2d());

    // All visible targets from the latest frame
    public TargetObservation[] targetObservations = new TargetObservation[] {};

    // IDs of visible AprilTags
    public int[] tagIds = new int[] {};

    // Pose estimates used by AdvantageKit pose fusion
    public PoseObservation[] poseObservations = new PoseObservation[] {};
  }

  /** ID + horizontal offset to one detected target. */
  public static record TargetObservation(int id, Rotation2d tx) {}

  /** One robot pose observation from vision. */
  public static record PoseObservation(
      double timestamp,
      Pose3d pose,
      double ambiguity,
      int tagCount,
      double averageTagDistance,
      PoseObservationType type) {}

  public static enum PoseObservationType {
    PHOTONVISION,
    MEGATAG_1,
    MEGATAG_2
  }

  public default void updateInputs(VisionIOInputs inputs) {}
}
