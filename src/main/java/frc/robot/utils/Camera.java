package frc.robot.utils;

import java.util.List;
import java.util.Optional;

import org.photonvision.EstimatedRobotPose;
import org.photonvision.PhotonCamera;
import org.photonvision.PhotonPoseEstimator;
import org.photonvision.targeting.PhotonPipelineResult;
import org.photonvision.targeting.PhotonTrackedTarget;

import edu.wpi.first.math.Matrix;
import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.math.numbers.N1;
import edu.wpi.first.math.numbers.N3;
import edu.wpi.first.wpilibj.Alert;
import edu.wpi.first.wpilibj.Alert.AlertType;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.Constants;

public class Camera {

  private static final double maxAmbiguity = 0.25;
  private static final Matrix<N3, N1> singleTagStdDevs = VecBuilder.fill(4, 4, 8);
  private static final Matrix<N3, N1> multiTagStdDevs = VecBuilder.fill(0.5, 0.5, 1);

  private final PhotonCamera camera;
  private final Transform3d robotToCamTransform;
  private final PhotonPoseEstimator poseEstimator;
  private final Alert connectedAlert;
  private Matrix<N3, N1> curStdDevs = singleTagStdDevs;

  public Camera(String name, Rotation3d robotToCamRotation, Translation3d robotToCamTranslation) {
    camera = new PhotonCamera(name);

    connectedAlert = new Alert("Camera \"" + name + "\" is not connected!", AlertType.kError);

    robotToCamTransform = new Transform3d(robotToCamTranslation, robotToCamRotation);

    poseEstimator = new PhotonPoseEstimator(Constants.fieldLayout, robotToCamTransform);
  }

  /**
   * Get the estimated robot pose. Updates the current robot pose estimation,
   * standard deviations, and flushes the
   * cache of results.
   *
   * @return Estimated pose.
   */
  public Optional<EstimatedRobotPose> getEstimatedGlobalPose() {
    if (camera.isConnected()) {
      return estimateGlobalPose();
    } else {
      connectedAlert.set(true);
    }

    return Optional.empty();
  }

  public Matrix<N3, N1> getCurStdDevs() {
    return curStdDevs;
  }

  /**
   * Attempts to produce a global robot pose estimate from the camera's unread
   * vision results.
   *
   * @return an {@code Optional<EstimatedRobotPose>} containing the last
   *         successful pose estimate
   *         produced from the processed results, or {@code Optional.empty()} if
   *         no valid estimate
   *         could be produced.
   */
  private Optional<EstimatedRobotPose> estimateGlobalPose() {
    List<PhotonPipelineResult> resultsList = camera.getAllUnreadResults();

    // Remove any results that have no targets or are high ambiguity
    resultsList.removeIf(result -> (!result.hasTargets() || result.getBestTarget().getPoseAmbiguity() >= maxAmbiguity));

    Optional<EstimatedRobotPose> visionEst = Optional.empty();
    for (var result : resultsList) {
      // Start with multi-tag estimation
      visionEst = poseEstimator.estimateCoprocMultiTagPose(result);

      // Fallback to lowest-ambiguity single-tag estimation
      if (visionEst.isEmpty()) {
        SmartDashboard.putString("Vision/" + camera.getName() + "/Pose Estimation Strategy", "Lowest Ambiguity Pose");
        visionEst = poseEstimator.estimateLowestAmbiguityPose(result);
      } else {
        SmartDashboard.putString("Vision/" + camera.getName() + "/Pose Estimation Strategy", "Coproc MultiTag Pose");
      }

      updateEstimationStdDevs(visionEst, result.getTargets());
    }

    resultsList.clear();

    return visionEst;
  }

  /**
   * Calculates new standard deviations This algorithm is a heuristic that creates
   * dynamic standard
   * deviations based on number of tags, estimation strategy, and distance from
   * the tags.
   * 
   * From
   * https://github.com/PhotonVision/photonvision/blob/77457219c72dfa1891e7fc35e22d7d908d0f05ae/photonlib-java-examples/poseest/src/main/java/frc/robot/Vision.java#L118C1-L165C6
   *
   * @param estimatedPose The estimated pose to guess standard deviations for.
   * @param targets       All targets in this camera frame
   */
  private void updateEstimationStdDevs(
      Optional<EstimatedRobotPose> estimatedPose, List<PhotonTrackedTarget> targets) {
    if (estimatedPose.isEmpty()) {
      // No pose input. Default to single-tag std devs
      curStdDevs = singleTagStdDevs;

    } else {
      // Pose present. Start running Heuristic
      var estStdDevs = singleTagStdDevs;
      int numTags = 0;
      double avgDist = 0;

      // Precalculation - see how many tags we found, and calculate an
      // average-distance metric
      for (var tgt : targets) {
        var tagPose = poseEstimator.getFieldTags().getTagPose(tgt.getFiducialId());
        if (tagPose.isEmpty())
          continue;
        numTags++;
        avgDist += tagPose
            .get()
            .toPose2d()
            .getTranslation()
            .getDistance(estimatedPose.get().estimatedPose.toPose2d().getTranslation());
      }

      if (numTags == 0) {
        // No tags visible. Default to single-tag std devs
        curStdDevs = singleTagStdDevs;
      } else {
        // One or more tags visible, run the full heuristic.
        avgDist /= numTags;
        // Decrease std devs if multiple targets are visible
        if (numTags > 1)
          estStdDevs = multiTagStdDevs;
        // Increase std devs based on (average) distance
        if (numTags == 1 && avgDist > 4)
          estStdDevs = VecBuilder.fill(Double.MAX_VALUE, Double.MAX_VALUE, Double.MAX_VALUE);
        else
          estStdDevs = estStdDevs.times(1 + (avgDist * avgDist / 30));
        curStdDevs = estStdDevs;
      }
    }
  }
}
