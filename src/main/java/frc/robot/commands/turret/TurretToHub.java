package frc.robot.commands.turret;

import static edu.wpi.first.units.Units.Degrees;
import static edu.wpi.first.units.Units.Feet;
import static edu.wpi.first.units.Units.Meters;
import static edu.wpi.first.units.Units.RPM;
import static edu.wpi.first.units.Units.Radians;
import static edu.wpi.first.units.Units.Seconds;

import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
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
import frc.robot.subsystems.IntakeSubsystem;
import frc.robot.subsystems.LEDSubsystem;
import frc.robot.subsystems.SwerveSubsystem;
import frc.robot.subsystems.TurretSubsystem;
import frc.robot.subsystems.LEDSubsystem.LEDState;
import swervelib.SwerveInputStream;
import frc.robot.subsystems.LEDSubsystem;

public class TurretToHub extends Command {

    // Setting variables for everything (constants, subsystems, motors)

    private static final double ACCEL_LIMIT_WHILE_SHOOTING = 1.2 * 0.02;
    private static final double VEL_LIMIT_WHILE_SHOOTING = 0.8;

    private static final double ROT_ACCEL_LIMIT_WHILE_SHOOTING = 1.2 * 0.02;
    private static final double ROT_VEL_LIMIT_WHILE_SHOOTING = 0.8;

    private static final double LEAD_TIME_LATENCY_SECONDS = 0.2;

    private static final double SPIN_LEAD_TIME_SECONDS = 0.02;
    
    private static final double MAX_ANGLE_ERROR_RADIANS = Degrees.of(5).in(Radians);
    private static final double MAX_RPM_ERROR = 30;

    private final TurretSubsystem shooter;
    private final SwerveSubsystem swerve;
    private final SwerveInputStream swerveInputFieldOriented;
    private final IndexerSubsystem indexer;
    private final IntakeSubsystem intake;

    private final LEDSubsystem led;

    private ChassisSpeeds lastSwerveSpeeds = new ChassisSpeeds();

    private boolean isSpunUp = false;

    public TurretToHub(TurretSubsystem shooter, SwerveSubsystem swerve, IndexerSubsystem indexer,IntakeSubsystem intake,
            SwerveInputStream swerveInputFieldOriented, LEDSubsystem led) {
        this.shooter = shooter;
        this.swerve = swerve;
        this.indexer = indexer;
        this.intake = intake;
        this.swerveInputFieldOriented = swerveInputFieldOriented;
        this.led = led;

        addRequirements(shooter, swerve, indexer, intake);
    }

    // Resets variables when the program starts
    @Override
    public void initialize() {
        lastSwerveSpeeds = swerve.getFieldVelocity();
        isSpunUp = false;
    }

    // Spins the turret and the hood to their respective target positions when activated
    @Override
    public void execute() {

        // Set properties for when the camera is disabled
        if (!(Robot.noCameraMode == NoCameraMode.DISABLED)) {
            runNoCameraShot();
            return;
        }

        // Limit the velocity and acceleration of the robot to help out shoot while moving
        ChassisSpeeds swerveSpeeds = swerveInputFieldOriented.get();
        swerveSpeeds = SwerveSubsystem.applyAccelLimit(lastSwerveSpeeds, swerveSpeeds, ACCEL_LIMIT_WHILE_SHOOTING, ROT_ACCEL_LIMIT_WHILE_SHOOTING);
        swerveSpeeds = SwerveSubsystem.applyVelocityLimit(swerveSpeeds, VEL_LIMIT_WHILE_SHOOTING, ROT_VEL_LIMIT_WHILE_SHOOTING);
        lastSwerveSpeeds = swerveSpeeds;

        // Variable for the turret's position relative to the field (Robor position + turret position relative to robor)
        Pose2d turretPose = shooter.getTurretPoseFromRobotPose(swerve.getPose());

        // Variables for our target's location & robot velocity
        Strategy.StrategyConfig strategyConfig = Strategy.getLocationTarget(swerve.getPose().getTranslation());
        Translation2d locationTarget = strategyConfig.targetLocation;
        Translation2d commandedRobotVelocity = new Translation2d(swerveSpeeds.vxMetersPerSecond,
                swerveSpeeds.vyMetersPerSecond);

        // Calculate the lead time we need based on distance
        // Adjust the constant based on system latency
        Distance distanceToLocationTarget = Meters.of(locationTarget.getDistance(turretPose.getTranslation()));
        double leadTimeSeconds = LEAD_TIME_LATENCY_SECONDS + shooter.timeUntilHit(distanceToLocationTarget.in(Feet)).in(Seconds);

        // Lead the target based on our current velocity
        locationTarget = locationTarget.minus(commandedRobotVelocity.times(leadTimeSeconds));
        distanceToLocationTarget = Meters.of(locationTarget.getDistance(turretPose.getTranslation()));

        SmartDashboard.putNumber("TurretToHub/distanceToTarget", distanceToLocationTarget.in(Feet));

        AngularVelocity targetShooterSpeed = shooter.targetShooterSpeed(distanceToLocationTarget.in(Feet));
        // AngularVelocity targetShooterSpeed = RPM.of(SmartDashboard.getNumber("Target RPM", 3300));
        shooter.setShooterSpeed(targetShooterSpeed);
        SmartDashboard.putNumber("TurretToHub/shooterSpeed", targetShooterSpeed.in(RPM));

        // If we're aiming for the hub, turn our turret to face: a. the hub, b. the hub, c. the sad reality we're gonna take it apart next season, d. the hub
        if (strategyConfig.strategyType == StrategyType.HUB_SHOT) {
            // shooter.setHoodAngle(Degrees.of(70));
            shooter.setHoodAngle(shooter.targetHoodAngle(distanceToLocationTarget.in(Feet)));
            SmartDashboard.putNumber("TurretToHub/hoodAngle", shooter.targetHoodAngle(distanceToLocationTarget.in(Feet)).in(Degrees));
        } else {
            shooter.setHoodAngle(Degrees.of(45));
        }

        Rotation2d predictedSwerveHeading = swerve.getPose().getRotation().plus(new Rotation2d(Radians.of(swerveSpeeds.omegaRadiansPerSecond).times(SPIN_LEAD_TIME_SECONDS)));

        double azimuthError = shooter.setAzimuthAngle(locationTarget.minus(turretPose.getTranslation()).getAngle().minus(predictedSwerveHeading).getRadians());

        SmartDashboard.putNumber("TurretToHub/Azimuth Error", azimuthError);

        Translation2d rawInputTranslation2d = new Translation2d(swerveSpeeds.vxMetersPerSecond, swerveSpeeds.vyMetersPerSecond);
        if (rawInputTranslation2d.getNorm() < 0.05 && Math.abs(swerveSpeeds.omegaRadiansPerSecond) < 0.05) {
            swerve.lock(); // if we're not trying to move, lock the wheels to prevent being pushed
        } else {
            swerve.driveFieldOriented(swerveSpeeds);
        }

        if (azimuthError < 20 && (isSpunUp || (shooter.shooterAtSpeed(targetShooterSpeed.in(RPM), MAX_RPM_ERROR)))) {
            isSpunUp = true;
            indexer.runIndexing();
            intake.setIntakeSpeed(0.75);
        } else {
            indexer.stopIndexing();
            intake.setIntakeSpeed(0);
        }
    }

    // When the program ends, stop everything
    @Override
    public void end(boolean interuppted) {
        shooter.stopShooter();
        indexer.stopIndexing();
        // shooter.setHoodAngle(Degrees.of(80));
        shooter.dropHood();
        shooter.setShooterSpeedRPM(2000);
        isSpunUp = false;
    }

    // YOUR JOB WILL NEVER BE FINISHED
    @Override
    public boolean isFinished() {
        return false;
    }

    // Determines how the turret will work when the camera is disabled
    private void runNoCameraShot() {
        double targetShooterSpeed = 0;

        // Settings for if the robot is close to or far from the hub
        if (Robot.noCameraMode == NoCameraMode.CLOSE_SHOT) {
            shooter.setHoodAngle(Degrees.of(70));
            targetShooterSpeed = 1965;
        } else if (Robot.noCameraMode == NoCameraMode.FAR_SHOT) {
            shooter.setHoodAngle(Degrees.of(63));
            targetShooterSpeed = 2234;
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
