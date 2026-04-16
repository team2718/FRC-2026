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
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.utils.Camera;

@Logged
public class VisionSubsystem extends SubsystemBase {
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
    List<Camera.VisionMeasurement> measurements = new ArrayList<>();

    for (Camera camera : cameras) {
      measurements.addAll(camera.getVisionMeasurements());
    }

    measurements.sort(Comparator.comparingDouble(measurement -> measurement.estimatedPose().timestampSeconds));

    for (Camera.VisionMeasurement measurement : measurements) {
      Pose2d measurementPose = measurement.estimatedPose().estimatedPose.toPose2d();

      lastEstimatedPose = measurement.estimatedPose().estimatedPose;
      swerve.addVisionMeasurement(measurementPose, measurement.estimatedPose().timestampSeconds, measurement.stdDevs());
    }
  }
}