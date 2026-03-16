package frc.robot.commands.turret;

import static edu.wpi.first.units.Units.Degrees;
import static edu.wpi.first.units.Units.Feet;
import static edu.wpi.first.units.Units.Meters;
import static edu.wpi.first.units.Units.RPM;
import static edu.wpi.first.units.Units.Radians;
import static edu.wpi.first.units.Units.Seconds;

import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.units.measure.Distance;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants;
import frc.robot.Robot;
import frc.robot.Strategy;
import frc.robot.Robot.NoCameraMode;
import frc.robot.Strategy.StrategyType;
import frc.robot.subsystems.IndexerSubsystem;
import frc.robot.subsystems.SwerveSubsystem;
import frc.robot.subsystems.TurretSubsystem;
import swervelib.SwerveInputStream;

public class TurretToHub extends Command {    
    private static final double MAX_ANGLE_ERROR_RADIANS = Degrees.of(5).in(Radians);
    private static final double MAX_RPM_ERROR = 30;

    private final TurretSubsystem shooter;
    private final SwerveSubsystem swerve;
    private final SwerveInputStream swerveInputFieldOriented;
    private final IndexerSubsystem indexer;

    private ChassisSpeeds lastSwerveSpeeds = new ChassisSpeeds();

    private ProfiledPIDController turnController = new ProfiledPIDController(10, 0, 0,
            new TrapezoidProfile.Constraints(10, 3));

    private boolean isSpunUp = false;

    public TurretToHub(TurretSubsystem shooter, SwerveSubsystem swerve, IndexerSubsystem indexer,
            SwerveInputStream swerveInputFieldOriented) {
        this.shooter = shooter;
        this.swerve = swerve;
        this.swerveInputFieldOriented = swerveInputFieldOriented;
        this.indexer = indexer;

        turnController.enableContinuousInput(-Math.PI, Math.PI);

        addRequirements(shooter, swerve, indexer);
    }

    @Override
    public void initialize() {
        lastSwerveSpeeds = swerve.getFieldVelocity();
        isSpunUp = false;

        turnController.reset(shooter.getTurretPoseFromRobotPose(swerve.getPose()).getRotation().getRadians());
    }

    // Spins the turret and the hood to their respective target positions when
    // activated
    @Override
    public void execute() {

        if (!(Robot.noCameraMode == NoCameraMode.DISABLED)) {
            runNoCameraShot();
            return;
        }

        // Limit the velocity and acceleration of the robot to help out shoot while
        // moving
        ChassisSpeeds swerveSpeeds = swerveInputFieldOriented.get();
        swerveSpeeds = SwerveSubsystem.applyAccelLimit(lastSwerveSpeeds, swerveSpeeds, Constants.OperatorConstants.ACCEL_LIMIT_WHILE_SHOOTING);
        swerveSpeeds = SwerveSubsystem.applyVelocityLimit(swerveSpeeds, Constants.OperatorConstants.VEL_LIMIT_WHILE_SHOOTING);
        lastSwerveSpeeds = swerveSpeeds;

        Pose2d turretPose = shooter.getTurretPoseFromRobotPose(swerve.getPose());

        Strategy.StrategyConfig strategyConfig = Strategy.getLocationTarget(swerve.getPose().getTranslation());
        Translation2d locationTarget = strategyConfig.targetLocation;
        Translation2d commandedRobotVelocity = new Translation2d(swerveSpeeds.vxMetersPerSecond,
                swerveSpeeds.vyMetersPerSecond);

        // Iteratively solve the shoot-while-moving lead problem.
        // Each iteration recomputes the flight time from the updated (led) distance,
        // then re-applies the lead until the distance converges.
        Distance distanceToLocationTarget = Meters.of(locationTarget.getDistance(turretPose.getTranslation()));
        for (int i = 0; i < 4; i++) {
            double leadTimeSeconds = Constants.TurretConstants.SHOT_LEAD_TIME_LATENCY_SECONDS + shooter.timeUntilHit(distanceToLocationTarget.in(Feet)).in(Seconds);
            Translation2d newLocationTarget = strategyConfig.targetLocation.minus(commandedRobotVelocity.times(leadTimeSeconds));
            Distance newDistance = Meters.of(newLocationTarget.getDistance(turretPose.getTranslation()));
            locationTarget = newLocationTarget;
            // Optional break-early logic
            // if (Math.abs(newDistance.in(Feet) - distanceToLocationTarget.in(Feet)) < 1e-6) {
            //     break;
            // }
            distanceToLocationTarget = newDistance;
        }
        distanceToLocationTarget = Meters.of(locationTarget.getDistance(turretPose.getTranslation()));

        SmartDashboard.putNumber("TurretToHub/distanceToTarget", distanceToLocationTarget.in(Feet));

        AngularVelocity targetShooterSpeed = shooter.targetShooterSpeed(distanceToLocationTarget.in(Feet));
        // AngularVelocity targetShooterSpeed = RPM.of(SmartDashboard.getNumber("Target RPM", 2000));
        shooter.setShooterSpeed(targetShooterSpeed);
        SmartDashboard.putNumber("TurretToHub/shooterSpeed", targetShooterSpeed.in(RPM));

        if (strategyConfig.strategyType == StrategyType.HUB_SHOT) {
            Angle targetHoodAngle = shooter.targetHoodAngle(distanceToLocationTarget.in(Feet));
            shooter.setHoodAngle(targetHoodAngle);
            SmartDashboard.putNumber("TurretToHub/hoodAngle", targetHoodAngle.in(Degrees));
        } else {
            shooter.setHoodAngle(Degrees.of(45));
        }

        SmartDashboard.putNumber("TurretToHub/angleError", turnController.getPositionError());

        swerveSpeeds.omegaRadiansPerSecond = turnController.calculate(
                turretPose.getRotation().getRadians(),
                locationTarget.minus(turretPose.getTranslation()).getAngle().getRadians());

        if (isSpunUp && commandedRobotVelocity.getNorm() < 0.1 && Math.abs(swerveSpeeds.omegaRadiansPerSecond) < 0.01) {
            swerve.lock(); // if we're not trying to move, lock the wheels to prevent being pushed
        } else {
            swerve.driveFieldOriented(swerveSpeeds);
        }

        if (isSpunUp || (shooter.shooterAtSpeed(targetShooterSpeed.in(RPM), MAX_RPM_ERROR) && Math.abs(turnController.getPositionError()) < MAX_ANGLE_ERROR_RADIANS)) {
            isSpunUp = true;
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
        isSpunUp = false;
    }

    @Override
    public boolean isFinished() {
        return false;
    }

    private void runNoCameraShot() {
        swerve.lock();
        double targetShooterSpeed = 0;

        if (Robot.noCameraMode == NoCameraMode.CLOSE_SHOT) {
            shooter.setHoodAngle(Degrees.of(70));
            targetShooterSpeed = 1965;
        } else if (Robot.noCameraMode == NoCameraMode.FAR_SHOT) {
            shooter.setHoodAngle(Degrees.of(63));
            targetShooterSpeed = 2234;
        }

        shooter.setShooterSpeed(RPM.of(targetShooterSpeed));

        if (isSpunUp || (shooter.shooterAtSpeed(targetShooterSpeed, MAX_RPM_ERROR))) {
            isSpunUp = true;
            indexer.runIndexing();
        } else {
            indexer.stopIndexing();
        }
    }
}
