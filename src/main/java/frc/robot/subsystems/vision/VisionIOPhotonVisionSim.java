package frc.robot.subsystems.vision;

import static frc.robot.subsystems.vision.VisionConstants.aprilTagLayout;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Transform3d;
import java.util.LinkedList;
import java.util.List;
import java.util.function.Supplier;
import org.photonvision.PhotonCamera;
import org.photonvision.PhotonPoseEstimator;
import org.photonvision.PhotonPoseEstimator.PoseStrategy;
import org.photonvision.simulation.PhotonCameraSim;
import org.photonvision.simulation.SimCameraProperties;
import org.photonvision.simulation.VisionSystemSim;
import org.photonvision.targeting.PhotonPipelineResult;

public class VisionIOPhotonVisionSim implements VisionIO {
  private static VisionSystemSim visionSim;

  private final PhotonCamera camera;
  private final PhotonCameraSim cameraSim;
  private final Transform3d robotToCamera;
  private final PhotonPoseEstimator poseEstimator;
  private final Supplier<Pose2d> robotPoseSupplier;

  public VisionIOPhotonVisionSim(
      String cameraName, Transform3d robotToCamera, Supplier<Pose2d> robotPoseSupplier) {
    this.camera = new PhotonCamera(cameraName);
    this.robotToCamera = robotToCamera;
    this.robotPoseSupplier = robotPoseSupplier;

    if (visionSim == null) {
      visionSim = new VisionSystemSim("main");
      visionSim.addAprilTags(aprilTagLayout);
    }

    SimCameraProperties cameraProps = new SimCameraProperties();
    cameraProps.setCalibration(960, 720, Rotation2d.fromDegrees(90.0));
    cameraProps.setCalibError(0.35, 0.10);
    cameraProps.setFPS(30);
    cameraProps.setAvgLatencyMs(35);
    cameraProps.setLatencyStdDevMs(5);

    cameraSim = new PhotonCameraSim(camera, cameraProps);
    visionSim.addCamera(cameraSim, robotToCamera);

    poseEstimator =
        new PhotonPoseEstimator(
            aprilTagLayout, PoseStrategy.MULTI_TAG_PNP_ON_COPROCESSOR, robotToCamera);
    poseEstimator.setMultiTagFallbackStrategy(PoseStrategy.LOWEST_AMBIGUITY);
  }

  @Override
  public void updateInputs(VisionIOInputs inputs) {
    visionSim.update(robotPoseSupplier.get());

    inputs.connected = true;
    inputs.latestTargetObservation = new TargetObservation(-1, new Rotation2d());
    inputs.targetObservations = new TargetObservation[] {};
    inputs.tagIds = new int[] {};
    inputs.poseObservations = new PoseObservation[] {};

    PhotonPipelineResult result = camera.getLatestResult();

    List<TargetObservation> targetObservations = new LinkedList<>();
    List<Integer> tagIds = new LinkedList<>();
    List<PoseObservation> poseObservations = new LinkedList<>();

    if (result.hasTargets()) {
      var bestTarget = result.getBestTarget();
      inputs.latestTargetObservation =
          new TargetObservation(
              bestTarget.getFiducialId(), Rotation2d.fromDegrees(bestTarget.getYaw()));

      for (var target : result.getTargets()) {
        int id = target.getFiducialId();
        Rotation2d tx = Rotation2d.fromDegrees(target.getYaw());

        targetObservations.add(new TargetObservation(id, tx));

        if (id >= 0) {
          tagIds.add(id);
        }
      }
    }

    var estimatedPose = poseEstimator.update(result);
    if (estimatedPose.isPresent()) {
      var estimate = estimatedPose.get();

      double totalDistance = 0.0;
      int validTagCount = 0;
      double ambiguity = 0.0;

      for (var target : result.getTargets()) {
        if (target.getFiducialId() >= 0) {
          totalDistance += target.getBestCameraToTarget().getTranslation().getNorm();
          validTagCount++;
        }
      }

      if (result.hasTargets() && result.getTargets().size() == 1) {
        ambiguity = result.getBestTarget().getPoseAmbiguity();
      }

      double averageTagDistance = validTagCount > 0 ? totalDistance / validTagCount : 0.0;

      poseObservations.add(
          new PoseObservation(
              estimate.timestampSeconds,
              estimate.estimatedPose,
              ambiguity,
              validTagCount,
              averageTagDistance,
              PoseObservationType.PHOTONVISION));
    }

    inputs.targetObservations = targetObservations.toArray(new TargetObservation[0]);
    inputs.tagIds = tagIds.stream().mapToInt(Integer::intValue).toArray();
    inputs.poseObservations = poseObservations.toArray(new PoseObservation[0]);
  }
}
