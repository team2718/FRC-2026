package frc.robot.utils;

import java.util.ArrayList;
import java.util.List;
import java.util.Optional;

import org.photonvision.EstimatedRobotPose;
import org.photonvision.PhotonCamera;
import org.photonvision.PhotonPoseEstimator;
import org.photonvision.targeting.PhotonPipelineResult;
import org.photonvision.targeting.PhotonTrackedTarget;

import edu.wpi.first.math.Matrix;
import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.math.numbers.N1;
import edu.wpi.first.math.numbers.N3;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.networktables.StructPublisher;
import edu.wpi.first.wpilibj.Alert;
import edu.wpi.first.wpilibj.Alert.AlertType;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.Constants;

public class Camera {
  private static final double maxAmbiguity = 0.15;
  private static final double maxSingleTagDistanceMeters = 4.0;
  private static final Matrix<N3, N1> singleTagStdDevs = VecBuilder.fill(4, 4, 8);
  private static final Matrix<N3, N1> multiTagStdDevs = VecBuilder.fill(0.2, 0.2, 1);

  private final PhotonCamera camera;
  private final Transform3d robotToCamTransform;
  private final PhotonPoseEstimator poseEstimator;
  private final Alert connectedAlert;

  StructPublisher<Pose2d> robotPosePublisher;

  public static record VisionMeasurement(
      String cameraName,
      EstimatedRobotPose estimatedPose,
      Matrix<N3, N1> stdDevs,
      int tagCount,
      double averageTagDistanceMeters,
      String strategy) {
  }

  private static record TagStatistics(int tagCount, double averageTagDistanceMeters) {
  }

  public Camera(String name, Rotation3d robotToCamRotation, Translation3d robotToCamTranslation) {
    camera = new PhotonCamera(name);

    connectedAlert = new Alert("Camera \"" + name + "\" is not connected!", AlertType.kError);

    robotToCamTransform = new Transform3d(robotToCamTranslation, robotToCamRotation);

    poseEstimator = new PhotonPoseEstimator(Constants.fieldLayout, robotToCamTransform);

    // Create pose publisher for this camera with the camera name
    robotPosePublisher = NetworkTableInstance.getDefault().getStructTopic("Vision/" + name + "/Estimated Pose", Pose2d.struct).publish();
  }

  public List<VisionMeasurement> getVisionMeasurements() {
    if (!camera.isConnected()) {
      connectedAlert.set(true);
      return List.of();
    }

    connectedAlert.set(false);
    return estimateGlobalPoses();
  }

  private List<VisionMeasurement> estimateGlobalPoses() {
    List<PhotonPipelineResult> resultsList = camera.getAllUnreadResults();
    List<VisionMeasurement> measurements = new ArrayList<>();

    // Remove any results that have no targets or are high ambiguity
    resultsList.removeIf(result -> (!result.hasTargets() || result.getBestTarget().getArea() < 0.08 || result.getBestTarget().getPoseAmbiguity() >= maxAmbiguity));

    for (var result : resultsList) {
      Optional<EstimatedRobotPose> visionEst = poseEstimator.estimateCoprocMultiTagPose(result);
      String estimationStrategy = "Coproc MultiTag Pose";

      if (visionEst.isEmpty()) {
        visionEst = poseEstimator.estimateLowestAmbiguityPose(result);
        estimationStrategy = "Lowest Ambiguity Pose";
      }

      if (visionEst.isEmpty()) {
        SmartDashboard.putString("Vision/" + camera.getName() + "/Pose Estimation Strategy", "No Valid Pose");
        continue;
      }

      TagStatistics tagStatistics = getTagStatistics(visionEst.get().estimatedPose, result.getTargets());

      if (isFarSingleTagEstimate(tagStatistics)) {
        SmartDashboard.putString("Vision/" + camera.getName() + "/Pose Estimation Strategy", "Rejected Far Single Tag");
        continue;
      }

      Matrix<N3, N1> stdDevs = getEstimationStdDevs(tagStatistics);

      robotPosePublisher.set(visionEst.get().estimatedPose.toPose2d());

      SmartDashboard.putString("Vision/" + camera.getName() + "/Pose Estimation Strategy", estimationStrategy);
      measurements.add(new VisionMeasurement(
          camera.getName(),
          visionEst.get(),
          stdDevs,
          tagStatistics.tagCount(),
          tagStatistics.averageTagDistanceMeters(),
          estimationStrategy));
    }

    return measurements;
  }

  private Matrix<N3, N1> getEstimationStdDevs(TagStatistics tagStatistics) {
    if (tagStatistics.tagCount() == 0) {
      return singleTagStdDevs;
    }

    var estStdDevs = tagStatistics.tagCount() > 1 ? multiTagStdDevs : singleTagStdDevs;
    double avgDist = tagStatistics.averageTagDistanceMeters();

    if (tagStatistics.tagCount() == 1 && avgDist > maxSingleTagDistanceMeters) {
      return VecBuilder.fill(Double.MAX_VALUE, Double.MAX_VALUE, Double.MAX_VALUE);
    }

    return estStdDevs.times(1 + (avgDist * avgDist / 30));
  }

  private TagStatistics getTagStatistics(Pose3d estimatedPose, List<PhotonTrackedTarget> targets) {
    int numTags = 0;
    double avgDist = 0;

    for (var tgt : targets) {
      var tagPose = poseEstimator.getFieldTags().getTagPose(tgt.getFiducialId());
      if (tagPose.isEmpty())
        continue;

      numTags++;
      avgDist += tagPose
          .get()
          .toPose2d()
          .getTranslation()
          .getDistance(estimatedPose.toPose2d().getTranslation());
    }

    if (numTags == 0) {
      return new TagStatistics(0, 0);
    }

    avgDist /= numTags;
    return new TagStatistics(numTags, avgDist);
  }

  private boolean isFarSingleTagEstimate(TagStatistics tagStatistics) {
    return tagStatistics.tagCount() == 1
        && tagStatistics.averageTagDistanceMeters() > maxSingleTagDistanceMeters;
  }
}
