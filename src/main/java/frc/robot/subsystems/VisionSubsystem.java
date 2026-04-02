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

  @Logged(name = "Last Estimated Pose")
  private Pose3d lastEstimatedPose = new Pose3d();

  public void updateSwervePoseFromVision(SwerveDrive swerveDrive) {
    Optional<EstimatedRobotPose> BackSideAprilCameraPose = BackSideAprilCamera.getEstimatedGlobalPose();

    if (BackSideAprilCameraPose.isPresent()) {
      EstimatedRobotPose pose = BackSideAprilCameraPose.get();

      lastEstimatedPose = pose.estimatedPose;

      swerveDrive.addVisionMeasurement(pose.estimatedPose.toPose2d(), pose.timestampSeconds,
          BackSideAprilCamera.getCurStdDevs());
    }

    Optional<EstimatedRobotPose> RightSideAprilCameraPose = RightSideAprilCamera.getEstimatedGlobalPose();

    if (RightSideAprilCameraPose.isPresent()) {
      EstimatedRobotPose pose = RightSideAprilCameraPose.get();

      lastEstimatedPose = pose.estimatedPose;

      swerveDrive.addVisionMeasurement(pose.estimatedPose.toPose2d(), pose.timestampSeconds,
          RightSideAprilCamera.getCurStdDevs());
    }
  }
}