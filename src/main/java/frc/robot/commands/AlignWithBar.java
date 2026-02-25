package frc.robot.commands;

import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj2.command.Command;
import swervelib.SwerveInputStream;
import frc.robot.subsystems.SwerveSubsystem;

public class AlignWithBar  extends Command{

    public static double getWrappedAngleDifference(double source, double target) {
        double diff = (target - source) % 360;

        if (diff > 180) {
            diff -= 360;
        }
        else if (diff <= -180) {
            diff += 360;
        }

        return diff;
    }

    SwerveSubsystem swerve;
    SwerveInputStream swerveInput;
    private final Translation2d barFrontLocation = new Translation2d(2.5, 5);
    

    public AlignWithBar(SwerveSubsystem swerve, SwerveInputStream swerveInput) {
        this.swerve = swerve;
        this.swerveInput = swerveInput;
        addRequirements(swerve);

    }

    public double clamp(double min, double max, double value) {
        if (value < min) {
            return min;
        }
        if (value > max) {
            return max;
        }
        return value;
    }


    @Override
    public void execute() {
        double angleFromTag15 = barFrontLocation.minus(swerve.getPose().getTranslation()).getAngle().getDegrees();

        double turnSpeed = getWrappedAngleDifference(swerve.getPose().getRotation().getDegrees(), angleFromTag15) * 0.05;

        turnSpeed = clamp(-0.8, 0.8, turnSpeed);

        swerve.driveFieldOriented(new ChassisSpeeds(swerveInput.get().vxMetersPerSecond, swerveInput.get().vyMetersPerSecond, turnSpeed));
    }
    
}
