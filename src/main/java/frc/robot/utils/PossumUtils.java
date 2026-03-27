package frc.robot.utils;

public class PossumUtils {
    public static double clamp(double value, double min, double max) {
        return Math.max(min, Math.min(max, value));
    }

    public static double getWrappedAngleDifference(double source, double target) {
        double diff = (target - source) % 360;

        if (diff >= 180) {
            diff -= 360;
        } else if (diff <= -180) {
            diff += 360;
        }

        return diff;
    }
}