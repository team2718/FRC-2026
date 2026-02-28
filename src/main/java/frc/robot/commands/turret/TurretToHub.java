package frc.robot.commands.turret;

import static edu.wpi.first.units.Units.Degrees;
import static edu.wpi.first.units.Units.Feet;
import static edu.wpi.first.units.Units.Meters;
import static edu.wpi.first.units.Units.RPM;

import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.units.measure.Distance;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants;
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

        Translation2d hubLocation = Constants.FieldConstants.hubCenterLocation;
        ChassisSpeeds robotChassisSpeeds = lastSwerveSpeeds;
        Translation2d robotVelocity = new Translation2d(robotChassisSpeeds.vxMetersPerSecond, robotChassisSpeeds.vyMetersPerSecond);
        hubLocation = hubLocation.minus(robotVelocity.times(1.3)); // Lead the target by 100ms based on current velocity

        Distance distance = Meters.of(hubLocation.getDistance(swerve.getPose().getTranslation()));

        SmartDashboard.putNumber("Distance to Hub (Feet)", distance.in(Feet));
        SmartDashboard.putNumber("Target Hood Angle (Degrees)", shooter.targetHoodAngle(distance.in(Feet)).in(Degrees));
        SmartDashboard.putNumber("Target Shooter Speed (RPM)", shooter.targetShooterSpeed(distance.in(Feet)).in(RPM));

        shooter.setHoodAngle(shooter.targetHoodAngle(distance.in(Feet)));
        shooter.setShooterSpeed(shooter.targetShooterSpeed(distance.in(Feet)));

        ChassisSpeeds swerveSpeeds = swerveInput.get();

        double angleError = getWrappedAngleDifference(
                swerve.getPose().getRotation().getDegrees(),
                hubLocation.minus(swerve.getPose().getTranslation()).getAngle()
                        .getDegrees() + 85);

        double turnSpeed = clamp(-3.0, 3.0, angleError * 0.15);

        swerveSpeeds.omegaRadiansPerSecond = turnSpeed;
        swerveSpeeds.vxMetersPerSecond = swerveSpeeds.vxMetersPerSecond * 0.15 + lastSwerveSpeeds.vxMetersPerSecond * 0.85;
        swerveSpeeds.vyMetersPerSecond = swerveSpeeds.vyMetersPerSecond * 0.15 + lastSwerveSpeeds.vyMetersPerSecond * 0.85;
        double swerveSpeedMagnitude = Math.hypot(swerveSpeeds.vxMetersPerSecond, swerveSpeeds.vyMetersPerSecond);
        if (swerveSpeedMagnitude > MAX_SPEED_WHILE_SHOOTING) {
            swerveSpeeds.vxMetersPerSecond = swerveSpeeds.vxMetersPerSecond / swerveSpeedMagnitude * MAX_SPEED_WHILE_SHOOTING;
            swerveSpeeds.vyMetersPerSecond = swerveSpeeds.vyMetersPerSecond / swerveSpeedMagnitude * MAX_SPEED_WHILE_SHOOTING;
        }
        lastSwerveSpeeds = swerveSpeeds;

        swerve.driveFieldOriented(swerveSpeeds);

        if (shooter.shooterAtSpeed() && Math.abs(angleError) < 15.0) {
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
