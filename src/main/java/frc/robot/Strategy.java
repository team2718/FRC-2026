package frc.robot;

import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj.DriverStation;

public class Strategy {

    public static enum StrategyType {
        HUB_SHOT,
        PASS;
    }

    public static class StrategyConfig {
        public StrategyType strategyType;
        public Translation2d targetLocation;

        public StrategyConfig(StrategyType strategyType, Translation2d targetLocation) {
            this.strategyType = strategyType;
            this.targetLocation = targetLocation;
        }
    }

    public static final Translation2d redHubLocation = new Translation2d(
            Constants.fieldLayout.getTagPose(5).get().getTranslation().getX(),
            Constants.fieldLayout.getTagPose(10).get().getTranslation().getY());
    public static final Translation2d blueHubLocation = new Translation2d(
            Constants.fieldLayout.getTagPose(18).get().getTranslation().getX(),
            Constants.fieldLayout.getTagPose(20).get().getTranslation().getY());

    public static StrategyConfig getLocationTarget(Translation2d robotLocation) {
        if (robotLocation == null) {
            // If we don't have a robot location, default to red hub location
            return new StrategyConfig(StrategyType.HUB_SHOT, redHubLocation);
        }

        DriverStation.Alliance alliance;
        try {
            alliance = DriverStation.getAlliance().get();
        } catch (Exception e) {
            // If we can't get the alliance, default to red hub location
            return new StrategyConfig(StrategyType.HUB_SHOT, redHubLocation);
        }
        
        if (alliance == DriverStation.Alliance.Red) {
            if (robotLocation.getX() > redHubLocation.getX()) { // Are we left of the hub?
                return new StrategyConfig(StrategyType.HUB_SHOT, redHubLocation);
            } else {
                if (robotLocation.getY() < redHubLocation.getY()) { // Shoot a bit past the nearest ramp
                    return new StrategyConfig(StrategyType.PASS, redHubLocation.plus(new Translation2d(3, -2)));
                } else {
                    return new StrategyConfig(StrategyType.PASS, redHubLocation.plus(new Translation2d(3, 2)));
                }
            }
        } else {
            if (robotLocation.getX() < blueHubLocation.getX()) { // Are we right of the hub?
                return new StrategyConfig(StrategyType.HUB_SHOT, blueHubLocation);
            } else {
                if (robotLocation.getY() < blueHubLocation.getY()) { // Shoot a bit past the nearest ramp
                    return new StrategyConfig(StrategyType.PASS, blueHubLocation.plus(new Translation2d(-3, -2)));
                } else {
                    return new StrategyConfig(StrategyType.PASS, blueHubLocation.plus(new Translation2d(-3, 2)));
                }
            }
        }
    }

}
