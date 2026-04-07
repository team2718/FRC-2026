package frc.robot.subsystems;

import java.util.ArrayList;
import java.util.Comparator;
import java.util.List;

import edu.wpi.first.epilogue.Logged;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.utils.Camera;

@Logged
public class VisionSubsystem extends SubsystemBase {
  private static final double maxSingleTagPoseErrorMeters = 1.0;
  private static final double maxMultiTagPoseErrorMeters = 2.0;

  private final Camera BackSideAprilCamera = new Camera("BackSideAprilCamera",
      new Rotation3d(
          Units.degreesToRadians(0),
          Units.degreesToRadians(-22),
          Units.degreesToRadians(-170)),
      new Translation3d(
          Units.inchesToMeters(-12.61),
          Units.inchesToMeters(-11.01),
          Units.inchesToMeters(15.69)));

    private final Camera RightSideAprilCamera = new Camera("RightSideAprilCamera",
      new Rotation3d(
          Units.degreesToRadians(0),
          Units.degreesToRadians(-22),
          Units.degreesToRadians(-100)),
      new Translation3d(
          Units.inchesToMeters(-11.01),
          Units.inchesToMeters(-12.61),
          Units.inchesToMeters(15.69)));

  private final List<Camera> cameras = List.of(BackSideAprilCamera, RightSideAprilCamera);

  @Logged(name = "Last Estimated Pose")
  private Pose3d lastEstimatedPose = new Pose3d();

  public void updateSwervePoseFromVision(SwerveSubsystem swerve) {
    Pose2d currentPose = swerve.getPose();
    List<Camera.VisionMeasurement> measurements = new ArrayList<>();

    for (Camera camera : cameras) {
      measurements.addAll(camera.getVisionMeasurements(currentPose));
    }

    measurements.sort(Comparator.comparingDouble(measurement -> measurement.estimatedPose().timestampSeconds));

    int acceptedMeasurements = 0;
    int rejectedMeasurements = 0;

    for (Camera.VisionMeasurement measurement : measurements) {
      Pose2d measurementPose = measurement.estimatedPose().estimatedPose.toPose2d();

      if (!shouldAcceptMeasurement(currentPose, measurement)) {
        rejectedMeasurements++;
        SmartDashboard.putString("Vision/" + measurement.cameraName() + "/Last Rejection", "Pose Delta Too Large");
        continue;
      }

      lastEstimatedPose = measurement.estimatedPose().estimatedPose;
      swerve.addVisionMeasurement(measurementPose, measurement.estimatedPose().timestampSeconds, measurement.stdDevs());
      currentPose = swerve.getPose();
      acceptedMeasurements++;
    }

    SmartDashboard.putNumber("Vision/Accepted Measurements", acceptedMeasurements);
    SmartDashboard.putNumber("Vision/Rejected Measurements", rejectedMeasurements);
  }

  private boolean shouldAcceptMeasurement(Pose2d currentPose, Camera.VisionMeasurement measurement) {
    Pose2d measurementPose = measurement.estimatedPose().estimatedPose.toPose2d();
    double translationErrorMeters = currentPose.getTranslation().getDistance(measurementPose.getTranslation());
    double maxPoseErrorMeters = measurement.tagCount() > 1
        ? maxMultiTagPoseErrorMeters
        : maxSingleTagPoseErrorMeters;

    SmartDashboard.putNumber(
        "Vision/" + measurement.cameraName() + "/Translation Error Meters",
        translationErrorMeters);

    return translationErrorMeters <= maxPoseErrorMeters;
  }
}