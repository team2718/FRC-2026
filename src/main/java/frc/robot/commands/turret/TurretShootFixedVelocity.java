package frc.robot.commands.turret;

import static edu.wpi.first.units.Units.Degrees;
import static edu.wpi.first.units.Units.Feet;
import static edu.wpi.first.units.Units.Meters;
import static edu.wpi.first.units.Units.RPM;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.interpolation.InterpolatingDoubleTreeMap;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.units.measure.Distance;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.robot.Strategy;
import frc.robot.subsystems.IndexerSubsystem;
import frc.robot.subsystems.SwerveSubsystem;
import frc.robot.subsystems.TurretSubsystem;
import frc.robot.utils.PossumUtils;
import swervelib.SwerveInputStream;

public class TurretShootFixedVelocity extends Command {
    private final TurretSubsystem shooter;
    private final SwerveSubsystem swerve;
    private final SwerveInputStream swerveInput;
    private final IndexerSubsystem indexer;

    private ChassisSpeeds lastSwerveSpeeds = new ChassisSpeeds();
    private double MAX_SPEED_BETWEEN_UPDATES = 0.05;
    private double MAX_SPEED_WHILE_SHOOTING = 0.7;

    private InterpolatingDoubleTreeMap shooterSpeeds = new InterpolatingDoubleTreeMap();

    private InterpolatingDoubleTreeMap hoodAngles = new InterpolatingDoubleTreeMap();

    public TurretShootFixedVelocity(TurretSubsystem shooter, SwerveSubsystem swerve, IndexerSubsystem indexer,
            SwerveInputStream swerveInput) {
        this.shooter = shooter;
        this.swerve = swerve;
        this.swerveInput = swerveInput;
        this.indexer = indexer;

        // SmartDashboard.putNumber("Target RPM", 2000);

        addRequirements(shooter, swerve, indexer);

        //Shooter Speed Values
        shooterSpeeds.clear();
        shooterSpeeds.put(2.5,24.9);
        shooterSpeeds.put(5.0,23.3);
        shooterSpeeds.put(10.0,24.8);
        shooterSpeeds.put(15.0,27.4);
        shooterSpeeds.put(20.0,29.9);
        shooterSpeeds.put(25.0,32.0);
        shooterSpeeds.put(30.0,35.1);

        //Hood Angle Values
        hoodAngles.clear();
        hoodAngles.put(2.5,83.9);
        hoodAngles.put(5.0,78.4);
        hoodAngles.put(10.0,69.1);
        hoodAngles.put(15.0,64.2);
        hoodAngles.put(20.0,60.3);
        hoodAngles.put(25.0,53.8);
        hoodAngles.put(30.0,57.7);
        
    }

    @Override
    public void initialize() {
        lastSwerveSpeeds = swerve.getFieldVelocity();
    }

    // Spins the turret and the hood to their respective target positions when
    // activated
    @Override
    public void execute() {

        AngularVelocity angularVelocity = RPM.of(SmartDashboard.getNumber("Target RPM", 2000));

        Pose2d turretPose = shooter.getTurretPoseFromRobotPose(swerve.getPose());

        Strategy.StrategyConfig strategyConfig = Strategy.getLocationTarget(swerve.getPose().getTranslation());
        Translation2d locationTarget = strategyConfig.targetLocation;
        Translation2d robotVelocity = new Translation2d(lastSwerveSpeeds.vxMetersPerSecond,
                lastSwerveSpeeds.vyMetersPerSecond);

        // Move the target back by 1.3 seconds worth of velocity to allow for shooting
        // while moving
        // TODO: Calculate time based on distance to target (??)
        locationTarget = locationTarget.minus(robotVelocity.times(1.3));

        Distance distanceToLocationTarget = Meters.of(locationTarget.getDistance(turretPose.getTranslation()));

        SmartDashboard.putNumber("Distance For Testing", distanceToLocationTarget.in(Feet));

        double angleError = PossumUtils.getWrappedAngleDifference(
        turretPose.getRotation().getDegrees(),
        locationTarget.minus(turretPose.getTranslation()).getAngle().getDegrees());

        // shooter.setTurretAngle(shooter.targetTurretAngle(angleError));
        shooter.setHoodAngle(shooter.targetHoodAngle(hoodAngles.get(distanceToLocationTarget.in(Feet))));
        shooter.setShooterSpeed(RPM.of(shooterSpeeds.get(distanceToLocationTarget.in(Feet))));

        ChassisSpeeds swerveSpeeds = swerveInput.get();

        double turnSpeed = PossumUtils.clamp(-3.0, 3.0, angleError * 0.15);

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

        if (shooter.shooterAtSpeed(angularVelocity.in(RPM), 50) && Math.abs(angleError) < 15.0) {
            indexer.runIndexing(angularVelocity);
        } else {
            indexer.stopIndexing();
        }
    }

    @Override
    public void end(boolean interuppted) {
        indexer.stopIndexing();
        //shooter.stopShooter();
        //shooter.setHoodAngle(Degrees.of(80));
    }

    @Override
    public boolean isFinished() {
        return false;
    }
}
