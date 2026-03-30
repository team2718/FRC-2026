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
import frc.robot.subsystems.LEDSubsystem.LEDState;
import swervelib.SwerveInputStream;

public class TurretToHubAuto extends Command {

    // Setting variables for everything (constants, subsystems, motors)

    private static final double ACCEL_LIMIT_WHILE_SHOOTING = 1.2 * 0.02;
    private static final double VEL_LIMIT_WHILE_SHOOTING = 0.8;

    private static final double ROT_ACCEL_LIMIT_WHILE_SHOOTING = 1.2 * 0.02;
    private static final double ROT_VEL_LIMIT_WHILE_SHOOTING = 0.8;

    private static final double LEAD_TIME_LATENCY_SECONDS = 0.2;

    private static final double HEADING_LEAD_TIME_SECONDS = 0.02;
    
    private static final double MAX_RPM_ERROR = 30;

    private final TurretSubsystem shooter;
    private final SwerveSubsystem swerve;
    private final IndexerSubsystem indexer;
    private final IntakeSubsystem intake;

    private final LEDSubsystem led;

    private ChassisSpeeds lastSwerveSpeeds = new ChassisSpeeds();

    private boolean isSpunUp = false;

    private final double redTrenchX = 11.916;
    private final double blueTrenchX = 4.626;

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
        lastSwerveSpeeds = swerve.getFieldVelocity();
        isSpunUp = false;

        // Run intake while shooting
        intake.setIntakeSpeed(0.75);
    }

    // Spins the turret and the hood to their respective target positions when activated
    @Override
    public void execute() {

        // Set properties for when the camera is disabled
        if (!(Robot.noCameraMode == NoCameraMode.DISABLED)) {
            runNoCameraShot();
            swerve.lock();
            return;
        }

        // Limit the velocity and acceleration of the robot to help out shoot while moving
        ChassisSpeeds swerveSpeeds = swerve.getFieldVelocity();
        // swerveSpeeds = SwerveSubsystem.applyAccelLimit(lastSwerveSpeeds, swerveSpeeds, ACCEL_LIMIT_WHILE_SHOOTING, ROT_ACCEL_LIMIT_WHILE_SHOOTING);
        // swerveSpeeds = SwerveSubsystem.applyVelocityLimit(swerveSpeeds, VEL_LIMIT_WHILE_SHOOTING, ROT_VEL_LIMIT_WHILE_SHOOTING);
        lastSwerveSpeeds = swerveSpeeds;

        // Variables for our target's location & robot velocity
        Strategy.StrategyConfig strategyConfig = Strategy.getLocationTarget(swerve.getPose().getTranslation());

        SmartDashboard.putString("TurretToHubAuto/Strategy", strategyConfig.strategyType.toString());

        if (strategyConfig.strategyType == StrategyType.DONT_SHOOT) {
            // If we don't want to shoot, stop the indexer
            indexer.stopIndexing();
        } else {
            shootAtLocation(strategyConfig.targetLocation, swerveSpeeds, strategyConfig);
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
    private Translation2d leadShot(Translation2d originalLocationTarget, Translation2d commandedRobotVelocity, Pose2d turretPose) {
        Translation2d projectedLocationTarget = originalLocationTarget;
        Distance distanceToLocationTarget = Meters.of(originalLocationTarget.getDistance(turretPose.getTranslation()));
        
        for (int i = 0; i < 4; i++) {
            double leadTimeSeconds = LEAD_TIME_LATENCY_SECONDS + shooter.timeUntilHit(distanceToLocationTarget.in(Feet)).in(Seconds);            
            projectedLocationTarget = originalLocationTarget.minus(commandedRobotVelocity.times(leadTimeSeconds));
            distanceToLocationTarget = Meters.of(projectedLocationTarget.getDistance(turretPose.getTranslation()));
        }

        return projectedLocationTarget;
    }

    private void shootAtLocation(Translation2d locationTarget, ChassisSpeeds swerveSpeeds, Strategy.StrategyConfig strategyConfig) {
        Translation2d commandedRobotVelocity = new Translation2d(swerveSpeeds.vxMetersPerSecond, swerveSpeeds.vyMetersPerSecond);
        Pose2d turretPose = shooter.getTurretPoseFromRobotPose(swerve.getPose());

        // Lead the target based on our current velocity
        Translation2d ledLocationTarget = leadShot(locationTarget, commandedRobotVelocity, turretPose);

        Distance distanceToLocationTarget = Meters.of(ledLocationTarget.getDistance(turretPose.getTranslation()));
        SmartDashboard.putNumber("TurretToHub/Distance to Target Feet", distanceToLocationTarget.in(Feet));

        AngularVelocity targetShooterSpeed = shooter.targetShooterSpeed(distanceToLocationTarget.in(Feet));
        SmartDashboard.putNumber("TurretToHub/Target Shooter Speed RPM", targetShooterSpeed.in(RPM));

        // If we're aiming for the hub, turn our turret to face: a. the hub, b. the hub, c. the sad reality we're gonna take it apart next season, d. the hub
        Angle targetHoodAngle;

        if (strategyConfig.strategyType == StrategyType.HUB_SHOT) {
            shooter.setShooterSpeed(targetShooterSpeed);
            targetHoodAngle = shooter.targetHoodAngle(distanceToLocationTarget.in(Feet));
        } else {
            shooter.setShooterSpeed(RPM.of(2000 + distanceToLocationTarget.in(Feet) * 50)); // low-ish passing speed
            targetHoodAngle = Degrees.of(45);
        }

        // Check that we aren't near the trench so we don't accidentally hit the trench with the hood
        // Put hood down if we are within 0.5 meters of the trench
        if (Math.abs(swerve.getPose().getTranslation().getX() - redTrenchX) < 0.3 || Math.abs(swerve.getPose().getTranslation().getX() - blueTrenchX) < 0.3) {
            shooter.dropHood();
            indexer.stopIndexing();
            return;
        } else {
            shooter.setHoodAngle(targetHoodAngle);
        }

        SmartDashboard.putNumber("TurretToHub/Target Hood Angle Degrees", targetHoodAngle.in(Degrees));

        Rotation2d predictedSwerveHeading = swerve.getPose().getRotation().plus(new Rotation2d(Radians.of(swerveSpeeds.omegaRadiansPerSecond).times(HEADING_LEAD_TIME_SECONDS)));
        double azimuthErrorDegrees = shooter.setAzimuthAngle(ledLocationTarget.minus(turretPose.getTranslation()).getAngle().minus(predictedSwerveHeading).getMeasure());

        SmartDashboard.putNumber("TurretToHub/Azimuth Error Degrees", azimuthErrorDegrees);

        if (Math.abs(azimuthErrorDegrees) < 2 && (isSpunUp || (shooter.shooterAtSpeed(targetShooterSpeed.in(RPM), MAX_RPM_ERROR)))) {
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
