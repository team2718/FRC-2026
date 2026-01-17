import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.apriltag.AprilTagFields;
import edu.wpi.first.math.Matrix;
import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Transform2d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;

public class Vision {

     public static final AprilTagFieldLayout fieldLayout = AprilTagFieldLayout
      .loadField(AprilTagFields.k2026RebuiltWelded);
    public Pose2d robotPosition;


      public Vision(Field2d field) {
        this.field2d = field;
        this.robotPosition = new Pose2d();
      }
    
    public Translation3d findBalls () {
        return new Translation3d();

    }

    public Pose3d distanceFromAprilTag(int aprilTag){
         Optional<Pose3d> aprilTagPose3d = fieldLayout.getTagPose(aprilTag);
         if (aprilTagPose3d.isPresent()) {
            return aprilTagPose3d.get().transformBy(robotPosition);
         } else {
            throw new RuntimeException("Cannot get AprilTag " + aprilTag + " from field " + fieldLayout.toString());
        }

    }

     public void updatePoseEstimation() {

     }

    public Pose2d getPosition(){
        return this.robotPosition;
    }
}
