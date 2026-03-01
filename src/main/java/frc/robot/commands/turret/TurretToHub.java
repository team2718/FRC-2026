package frc.robot.commands.turret;

import static edu.wpi.first.units.Units.Degrees;
import static edu.wpi.first.units.Units.Feet;
import static edu.wpi.first.units.Units.Meters;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.units.measure.Distance;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Strategy;
import frc.robot.subsystems.IndexerSubsystem;
import frc.robot.subsystems.SwerveSubsystem;
import frc.robot.subsystems.TurretSubsystem;
import swervelib.SwerveInputStream;

public class TurretToHub extends Command {
    private final TurretSubsystem shooter;
    private final SwerveSubsystem swerve;
    private final SwerveInputStream swerveInput;
    private final IndexerSubsystem indexer;

    private ChassisSpeeds lastSwerveSpeeds = new ChassisSpeeds();
    private double MAX_SPEED_BETWEEN_UPDATES = 0.05;
    private double MAX_SPEED_WHILE_SHOOTING = 0.7;

    public TurretToHub(TurretSubsystem shooter, SwerveSubsystem swerve, IndexerSubsystem indexer,
            SwerveInputStream swerveInput) {
        this.shooter = shooter;
        this.swerve = swerve;
        this.swerveInput = swerveInput;
        this.indexer = indexer;
        addRequirements(shooter, swerve, indexer);
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
    public void initialize() {
        lastSwerveSpeeds = swerve.getFieldVelocity();
    }

    // Spins the turret and the hood to their respective target positions when
    // activated
    @Override
    public void execute() {

        Pose2d turretPose = shooter.getTurretPoseFromRobotPose(swerve.getPose());

        Translation2d locationTarget = Strategy.getLocationTarget(swerve.getPose().getTranslation());
        Translation2d robotVelocity = new Translation2d(lastSwerveSpeeds.vxMetersPerSecond,
                lastSwerveSpeeds.vyMetersPerSecond);

        // Move the target back by 1.3 seconds worth of velocity to allow for shooting
        // while moving
        // TODO: Calculate time based on distance to target (??)
        locationTarget = locationTarget.minus(robotVelocity.times(1.3));

        Distance distanceToLocationTarget = Meters.of(locationTarget.getDistance(turretPose.getTranslation()));

        shooter.setHoodAngle(shooter.targetHoodAngle(distanceToLocationTarget.in(Feet)));
        shooter.setShooterSpeed(shooter.targetShooterSpeed(distanceToLocationTarget.in(Feet)));

        ChassisSpeeds swerveSpeeds = swerveInput.get();

        double angleError = getWrappedAngleDifference(
                turretPose.getRotation().getDegrees(),
                locationTarget.minus(turretPose.getTranslation()).getAngle().getDegrees());

        double turnSpeed = clamp(-3.0, 3.0, angleError * 0.15);

        swerveSpeeds.omegaRadiansPerSecond = turnSpeed;

        swerveSpeeds = SwerveSubsystem.applyAccelLimit(lastSwerveSpeeds, swerveSpeeds, MAX_SPEED_BETWEEN_UPDATES);

        // Limit the speed while shooting to prevent overshooting the target
        double swerveSpeedMagnitude = Math.hypot(swerveSpeeds.vxMetersPerSecond, swerveSpeeds.vyMetersPerSecond);
        if (swerveSpeedMagnitude > MAX_SPEED_WHILE_SHOOTING) {
            swerveSpeeds.vxMetersPerSecond = swerveSpeeds.vxMetersPerSecond / swerveSpeedMagnitude
                    * MAX_SPEED_WHILE_SHOOTING;
            swerveSpeeds.vyMetersPerSecond = swerveSpeeds.vyMetersPerSecond / swerveSpeedMagnitude
                    * MAX_SPEED_WHILE_SHOOTING;
        }

        lastSwerveSpeeds = swerveSpeeds;

        swerve.driveFieldOriented(swerveSpeeds);

        if (shooter.shooterAtSpeed(300) && Math.abs(angleError) < 15.0) {
            indexer.runIndexing();
        } else {
            indexer.stopIndexing();
        }
    }

    @Override
    public void end(boolean interuppted) {
        shooter.stopShooter();
        indexer.stopIndexing();
        shooter.setHoodAngle(Degrees.of(80));
    }

    @Override
    public boolean isFinished() {
        return false;
    }
}
