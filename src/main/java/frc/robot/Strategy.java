package frc.robot;

import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj.DriverStation;

public class Strategy {

    public enum StrategyType {
        DONT_SHOOT,
        HUB_SHOT,
        PASS
    }

    public static class StrategyConfig {
        public StrategyType strategyType;
        public Translation2d targetLocation;

        public StrategyConfig(StrategyType strategyType) {
            this.strategyType = strategyType;
            this.targetLocation = null;
        }

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

    // Triangular DONT_SHOOT zone centered on the hub
    private static final double DONT_SHOOT_TIP_DIST  = 3.40; // X depth (meters) at which the zone closes to a point
    private static final double DONT_SHOOT_HUB_WIDTH = 1.32; // Y half-width (meters) of the zone at the hub's X boundary

    private static final double PASS_X_OFFSET = 3.00; // how deep into alliance zone to pass (meters)
    private static final double PASS_Y_OFFSET = 2.25; // how far from hub laterally to pass (meters)

    public static StrategyConfig getLocationTarget(Translation2d robotLocation) {
        if (robotLocation == null) {
            return new StrategyConfig(StrategyType.HUB_SHOT, redHubLocation);
        }

        DriverStation.Alliance alliance;
        try {
            alliance = DriverStation.getAlliance().get();
        } catch (Exception e) {
            return new StrategyConfig(StrategyType.HUB_SHOT, redHubLocation);
        }

        return alliance == DriverStation.Alliance.Red
                ? getStrategyForHub(robotLocation, redHubLocation, 1)
                : getStrategyForHub(robotLocation, blueHubLocation, -1);
    }

    // hubSideSign: +1 for Red (hub-shot side is at greater X), -1 for Blue
    // (hub-shot side is at lesser X)
    private static StrategyConfig getStrategyForHub(
            Translation2d robot, Translation2d hub, int hubSideSign) {

        // Coordinate where hub is at the origin and +X is towards our alliance side
        double xOffset = (robot.getX() - hub.getX()) * hubSideSign;
        double yOffset = robot.getY() - hub.getY();

        // If we're in our alliance zone, let's score
        if (xOffset > 0) {
            return new StrategyConfig(StrategyType.HUB_SHOT, hub);
        }

        // Pass if we are beyond our alliance zone unless we are within a triangle
        // around the hub where we can't get a good shot
        double dontShootThreshold = Math.max(0, DONT_SHOOT_HUB_WIDTH + xOffset * (DONT_SHOOT_HUB_WIDTH / DONT_SHOOT_TIP_DIST));

        if (Math.abs(yOffset) < dontShootThreshold) {
            return new StrategyConfig(StrategyType.DONT_SHOOT);
        }

        return new StrategyConfig(StrategyType.PASS, new Translation2d(
                hub.getX() + hubSideSign * PASS_X_OFFSET,
                hub.getY() + Math.copySign(PASS_Y_OFFSET, yOffset)));
    }

}
