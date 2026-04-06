package frc.robot.commands.turret;

import static edu.wpi.first.units.Units.Degrees;
import static edu.wpi.first.units.Units.Feet;
import static edu.wpi.first.units.Units.Meters;
import static edu.wpi.first.units.Units.RPM;
import static edu.wpi.first.units.Units.Radians;
import static edu.wpi.first.units.Units.Seconds;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.units.measure.Distance;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Robot;
import frc.robot.Strategy;
import frc.robot.Robot.NoCameraMode;
import frc.robot.Strategy.StrategyType;
import frc.robot.subsystems.IndexerSubsystem;
import frc.robot.subsystems.IntakeSubsystem;
import frc.robot.subsystems.LEDSubsystem;
import frc.robot.subsystems.SwerveSubsystem;
import frc.robot.subsystems.TurretSubsystem;
import frc.robot.utils.PossumUtils;
import frc.robot.subsystems.LEDSubsystem.LEDState;

public class TurretToHubAuto extends Command {

    // Setting variables for everything (constants, subsystems, motors)

    private static final double LEAD_TIME_LATENCY_SECONDS = 0.22; // translation lead time
    private static final double HEADING_LEAD_TIME_SECONDS = 0.15; // rotation lead time
    
    private static final double MAX_RPM_ERROR = 50;

    private final TurretSubsystem shooter;
    private final SwerveSubsystem swerve;
    private final IndexerSubsystem indexer;
    private final IntakeSubsystem intake;

    private final LEDSubsystem led;

    private boolean isSpunUp = false;

    private final double redTrenchX = 11.916;
    private final double blueTrenchX = 4.626;

    private Pose2d turretPose;

    public TurretToHubAuto(TurretSubsystem shooter, SwerveSubsystem swerve, IndexerSubsystem indexer, IntakeSubsystem intake, LEDSubsystem led) {
        this.shooter = shooter;
        this.swerve = swerve;
        this.indexer = indexer;
        this.intake = intake;
        this.led = led;

        addRequirements(shooter, indexer, intake);
    }

    // Resets variables when the program starts
    @Override
    public void initialize() {
        isSpunUp = false;

        // Run intake while shooting
        intake.setIntakeSpeed(0.75);
    }

    // Spins the turret and the hood to their respective target positions when activated
    @Override
    public void execute() {

        // Set properties for when the camera is disabled
        if (!(Robot.noCameraMode == NoCameraMode.DISABLED)) {
            // runNoCameraShot();
            swerve.lock();
            return;
        }

        turretPose = shooter.getTurretPoseFromRobotPose(swerve.getPose());

        Strategy.StrategyConfig strategyConfig = Strategy.getLocationTarget(turretPose.getTranslation());

        SmartDashboard.putString("TurretToHubAuto/Strategy", strategyConfig.strategyType.toString());

        if (strategyConfig.strategyType == StrategyType.DONT_SHOOT) {
            // If we don't want to shoot, stop the indexer
            indexer.stopIndexing();
        } else {
            shootAtLocation(strategyConfig, swerve.getFieldVelocity());
        }
    
        // // Command the swerve to drive, but if we're not trying to move, lock the wheels to prevent being pushed around
        // Translation2d rawInputTranslation2d = new Translation2d(swerveSpeeds.vxMetersPerSecond, swerveSpeeds.vyMetersPerSecond);
        // if (rawInputTranslation2d.getNorm() < 0.05 && Math.abs(swerveSpeeds.omegaRadiansPerSecond) < 0.05) {
        //     swerve.lock(); // if we're not trying to move, lock the wheels to prevent being pushed
        // } else {
        //     swerve.driveFieldOriented(swerveSpeeds);
        // }
    }

    // When the program ends, stop everything
    @Override
    public void end(boolean interuppted) {

        intake.setIntakeSpeed(0);
        indexer.stopIndexing();

        shooter.dropHood();
        shooter.setShooterSpeedRPM(2000);

        isSpunUp = false;
    }

    // YOUR JOB WILL NEVER BE FINISHED
    @Override
    public boolean isFinished() {
        return false;
    }

    // Lead the target based on our current velocity
    // Iteratively solve the shoot-while-moving lead problem.
    // Each iteration recomputes the flight time from the updated (led) distance,
    // then re-applies the lead until the distance converges.
    private Translation2d leadShot(Translation2d originalLocationTarget, Translation2d turretVelocity,
            Pose2d turretPose) {
        Translation2d projectedLocationTarget = originalLocationTarget;
        Distance distanceToLocationTarget = Meters.of(originalLocationTarget.getDistance(turretPose.getTranslation()));

        for (int i = 0; i < 4; i++) {
            double leadTimeSeconds = LEAD_TIME_LATENCY_SECONDS
                    + shooter.timeUntilHit(distanceToLocationTarget.in(Feet)).in(Seconds);
            projectedLocationTarget = originalLocationTarget.minus(turretVelocity.times(leadTimeSeconds));
            distanceToLocationTarget = Meters.of(projectedLocationTarget.getDistance(turretPose.getTranslation()));
        }

        return projectedLocationTarget;
    }


    private void shootAtLocation(Strategy.StrategyConfig strategyConfig, ChassisSpeeds swerveSpeeds) {

        Translation2d robotVelocity = new Translation2d(swerveSpeeds.vxMetersPerSecond, swerveSpeeds.vyMetersPerSecond);

        // account for the turret not being at the center of the robot
        // velocity direction is 90-degrees orthogonal to the translation
        Translation2d turretTranslation = turretPose.getTranslation();
        Translation2d turretVelocityDirection = new Translation2d(-turretTranslation.getY(), turretTranslation.getX());
        Translation2d turretVelocity = robotVelocity.plus(
                turretVelocityDirection.times(swerveSpeeds.omegaRadiansPerSecond));

        // Lead the target based on our current velocity
        Translation2d ledLocationTarget = leadShot(strategyConfig.targetLocation, turretVelocity, turretPose);

        // Publish the target and led target for debugging
        // targetPublisher.set(strategyConfig.targetLocation);
        // ledTargetPublisher.set(ledLocationTarget);

        // Get the distance to the target and the shooter speed we need
        Distance distanceToLocationTarget = Meters.of(ledLocationTarget.getDistance(turretPose.getTranslation()));
        AngularVelocity targetShooterSpeed = shooter.targetShooterSpeed(distanceToLocationTarget.in(Feet));

        SmartDashboard.putNumber("TurretToHub/Distance to Target Feet", distanceToLocationTarget.in(Feet));
        SmartDashboard.putNumber("TurretToHub/Target Shooter Speed RPM", targetShooterSpeed.in(RPM));

        // If we're aiming for the hub, turn our turret to face: a. the hub, b. the hub,
        // c. the sad reality we're gonna take it apart next season, d. the hub
        Angle targetHoodAngle;

        if (strategyConfig.strategyType == StrategyType.HUB_SHOT) {
            targetHoodAngle = shooter.targetHoodAngle(distanceToLocationTarget.in(Feet));
        } else {
            targetShooterSpeed = RPM.of(2000 + distanceToLocationTarget.in(Feet) * 100);
            targetHoodAngle = Degrees.of(45);
        }

        shooter.setShooterSpeed(targetShooterSpeed);

        // Check that we aren't near the trench so we don't accidentally hit the trench
        // with the hood
        // Put hood down if we are within 0.5 meters of the trench
        if (Math.abs(swerve.getPose().getTranslation().getX() - redTrenchX) < 0.4
                || Math.abs(swerve.getPose().getTranslation().getX() - blueTrenchX) < 0.4) {
            shooter.dropHood();
            indexer.stopIndexing();
            return;
        } else {
            shooter.setHoodAngle(targetHoodAngle);
        }

        SmartDashboard.putNumber("TurretToHub/Target Hood Angle Degrees", targetHoodAngle.in(Degrees));

        Rotation2d currentSwerveHeading = swerve.getPose().getRotation();
        Rotation2d futureSwerveHeading = currentSwerveHeading
                .plus(new Rotation2d(Radians.of(swerveSpeeds.omegaRadiansPerSecond).times(HEADING_LEAD_TIME_SECONDS)));
        Rotation2d rotation2dToTarget = ledLocationTarget.minus(turretPose.getTranslation()).getAngle();

        Rotation2d targetTurretRotation = rotation2dToTarget.minus(currentSwerveHeading);
        Rotation2d futureTargetTurretRotation = rotation2dToTarget.minus(futureSwerveHeading);

        Angle targetTurretAngle = PossumUtils.Rotation2dToAngle360(targetTurretRotation);
        Angle futureTargetTurretAngle = PossumUtils.Rotation2dToAngle360(futureTargetTurretRotation);

        SmartDashboard.putNumber("TurretToHub/Target Turret Angle Degrees", targetTurretAngle.in(Degrees));
        SmartDashboard.putNumber("TurretToHub/Future Target Turret Angle Degrees", futureTargetTurretAngle.in(Degrees));

        shooter.setAzimuthAngle(futureTargetTurretAngle);

        // Compute the current error by using the non-future-rotated target angle
        // We use the future prediction to compensate for latency only when we set the
        // target
        Angle currentTurretAngle = shooter.getTurretAngle();
        double azimuthErrorDegrees = targetTurretAngle.minus(currentTurretAngle).in(Degrees);

        SmartDashboard.putNumber("TurretToHub/Azimuth Error Degrees", azimuthErrorDegrees);

        if (Math.abs(azimuthErrorDegrees) < 10
                && (isSpunUp || (shooter.shooterAtSpeed(targetShooterSpeed.in(RPM), MAX_RPM_ERROR)))) {
            isSpunUp = true;
            indexer.runIndexing();
        } else {
            indexer.stopIndexing();
        }
    }

    // Determines how the turret will work when the camera is disabled
    private void runNoCameraShot() {
        double targetShooterSpeed = 0;

        // Settings for if the robot is close to or far from the hub
        if (Robot.noCameraMode == NoCameraMode.CLOSE_SHOT) {
            shooter.setHoodAngle(Degrees.of(70));
            targetShooterSpeed = 3340;
        } else if (Robot.noCameraMode == NoCameraMode.FAR_SHOT) {
            shooter.setHoodAngle(Degrees.of(63));
            targetShooterSpeed = 3800;
        }

        shooter.setShooterSpeed(RPM.of(targetShooterSpeed));

        // If the shooter is spun up, run the indexer
        if (isSpunUp || (shooter.shooterAtSpeed(targetShooterSpeed, MAX_RPM_ERROR))) {
            isSpunUp = true;
            indexer.runIndexing();
            //LED blue = shooting
            led.setLEDState(LEDState.BLUE);
        } else {
            indexer.stopIndexing();
            //LED green = ready to shoot
            led.setLEDState(LEDState.GREEN);

        }
    }
}
