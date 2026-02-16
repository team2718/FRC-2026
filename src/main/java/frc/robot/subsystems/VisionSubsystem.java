package frc.robot.subsystems;

import java.util.Optional;

import org.photonvision.EstimatedRobotPose;

import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.apriltag.AprilTagFields;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.utils.Camera;
import swervelib.SwerveDrive;

public class VisionSubsystem extends SubsystemBase {

  private final Camera frontCamera = new Camera("FrontAprilCamera",
      new Rotation3d(
          Units.degreesToRadians(0),
          Units.degreesToRadians(-50),
          Units.degreesToRadians(0)),
      new Translation3d(
          Units.inchesToMeters(12.97),
          Units.inchesToMeters(0),
          Units.inchesToMeters(4.25)));

  public static final AprilTagFieldLayout fieldLayout = AprilTagFieldLayout
      .loadField(AprilTagFields.k2026RebuiltWelded);

  public void updateSwervePoseFromVision(SwerveDrive swerveDrive) {
    Optional<EstimatedRobotPose> frontCameraPose = frontCamera.getEstimatedGlobalPose();

    if (frontCameraPose.isPresent()) {
      EstimatedRobotPose pose = frontCameraPose.get();
      swerveDrive.addVisionMeasurement(pose.estimatedPose.toPose2d(), pose.timestampSeconds,
          frontCamera.getCurStdDevs());
    }
  }
}