package frc.robot.subsystems;

import java.util.Optional;

import org.photonvision.EstimatedRobotPose;

import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.apriltag.AprilTagFields;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.utils.Camera;
import swervelib.SwerveDrive;

public class VisionSubsystem extends SubsystemBase {

  private final Camera frontCamera = new Camera("FrontAprilCamera",
      new Rotation3d(
          Units.degreesToRadians(0),
          Units.degreesToRadians(25),
          Units.degreesToRadians(95)),
      new Translation3d(
          Units.inchesToMeters(10.578),
          Units.inchesToMeters(-4),
          Units.inchesToMeters(28.8125)));

  public static final AprilTagFieldLayout fieldLayout = AprilTagFieldLayout
      .loadField(AprilTagFields.k2026RebuiltWelded);

  public Pose2d robotPosition;

  public VisionSubsystem() {
    this.robotPosition = new Pose2d();
  }

  public Translation3d findBalls() {
    return new Translation3d();

  }

  // public Pose3d distanceFromAprilTag(int aprilTag) {
  // Optional<Pose3d> aprilTagPose3d = fieldLayout.getTagPose(aprilTag);
  // if (aprilTagPose3d.isPresent()) {
  // return aprilTagPose3d.get().transformBy(robotPosition);
  // } else {
  // throw new RuntimeException("Cannot get AprilTag " + aprilTag + " from field "
  // + fieldLayout.toString());
  // }

  // }

  public Pose2d getPosition() {
    return this.robotPosition;
  }

  public void updatePoseFromTags(SwerveDrive swerveDrive) {
      Optional<EstimatedRobotPose> frontCameraPose = frontCamera.getEstimatedGlobalPose();

      if (frontCameraPose.isPresent()) {
        EstimatedRobotPose pose = frontCameraPose.get();
        swerveDrive.addVisionMeasurement(pose.estimatedPose.toPose2d(), pose.timestampSeconds, frontCamera.getCurStdDevs());
      }
  }
}
