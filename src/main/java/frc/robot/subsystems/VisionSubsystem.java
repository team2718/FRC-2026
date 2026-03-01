package frc.robot.subsystems;

import java.util.Optional;

import org.photonvision.EstimatedRobotPose;

import edu.wpi.first.epilogue.Logged;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.utils.Camera;
import swervelib.SwerveDrive;

@Logged
public class VisionSubsystem extends SubsystemBase {

  private final Camera frontCamera = new Camera("FrontAprilCamera",
      new Rotation3d(
          Units.degreesToRadians(0),
          Units.degreesToRadians(-50),
          Units.degreesToRadians(270)),
      new Translation3d(
          Units.inchesToMeters(-6.5),
          Units.inchesToMeters(-13.14),
          Units.inchesToMeters(16.13)));

  @Logged(name = "Last Estimated Pose")
  private Pose3d lastEstimatedPose = new Pose3d();

  public void updateSwervePoseFromVision(SwerveDrive swerveDrive) {
    Optional<EstimatedRobotPose> frontCameraPose = frontCamera.getEstimatedGlobalPose();

    if (frontCameraPose.isPresent()) {
      EstimatedRobotPose pose = frontCameraPose.get();

      // SmartDashboard.putNumber("Vision/Estimated Pose/X",
      // pose.estimatedPose.getX());
      // SmartDashboard.putNumber("Vision/Estimated Pose/Y",
      // pose.estimatedPose.getY());
      // SmartDashboard.putNumber("Vision/Estimated Pose/Z",
      // pose.estimatedPose.getZ());

      lastEstimatedPose = pose.estimatedPose;

      swerveDrive.addVisionMeasurement(pose.estimatedPose.toPose2d(), pose.timestampSeconds,
          frontCamera.getCurStdDevs());
    }
  }
}