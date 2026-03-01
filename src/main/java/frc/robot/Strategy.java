package frc.robot;

import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj.DriverStation;

public class Strategy {
    public static final Translation2d redHubLocation = new Translation2d(
            Constants.fieldLayout.getTagPose(5).get().getTranslation().getX(),
            Constants.fieldLayout.getTagPose(10).get().getTranslation().getY());
    public static final Translation2d blueHubLocation = new Translation2d(
            Constants.fieldLayout.getTagPose(18).get().getTranslation().getX(),
            Constants.fieldLayout.getTagPose(20).get().getTranslation().getY());

    public static Translation2d getLocationTarget(Translation2d robotLocation) {
        if (robotLocation == null) {
            // If we don't have a robot location, default to red hub location
            return redHubLocation;
        }

        DriverStation.Alliance alliance;
        try {
            alliance = DriverStation.getAlliance().get();
        } catch (Exception e) {
            // If we can't get the alliance, default to red hub location
            return redHubLocation;
        }
        
        if (alliance == DriverStation.Alliance.Red) {
            if (robotLocation.getX() > redHubLocation.getX()) { // Are we left of the hub?
                return redHubLocation;
            } else {
                if (robotLocation.getY() < redHubLocation.getY()) { // Shoot a bit past the nearest ramp
                    return redHubLocation.plus(new Translation2d(0.5, -2));
                } else {
                    return redHubLocation.plus(new Translation2d(0.5, 2));
                }
            }
        } else {
            if (robotLocation.getX() < blueHubLocation.getX()) { // Are we right of the hub?
                return blueHubLocation;
            } else {
                if (robotLocation.getY() < blueHubLocation.getY()) { // Shoot a bit past the nearest ramp
                    return blueHubLocation.plus(new Translation2d(-0.5, -2));
                } else {
                    return blueHubLocation.plus(new Translation2d(-0.5, 2));
                }
            }
        }
    }

}
