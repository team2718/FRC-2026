package frc.robot.utils;

import static edu.wpi.first.units.Units.Degrees;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.units.measure.Angle;

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

    public static Angle Rotation2dToAngle360(Rotation2d rotation) {
        double angle = rotation.getDegrees() % 360;
        if (angle < 0) {
            angle += 360;
        }
        return Degrees.of(angle);
    }
}