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
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.units.measure.Distance;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Robot;
import frc.robot.Strategy;
import frc.robot.Robot.NoCameraMode;
import frc.robot.Strategy.StrategyType;
import frc.robot.subsystems.IndexerSubsystem;
import frc.robot.subsystems.LEDSubsystem;
import frc.robot.subsystems.SwerveSubsystem;
import frc.robot.subsystems.TurretSubsystem;
import frc.robot.subsystems.LEDSubsystem.LEDState;
import swervelib.SwerveInputStream;

public class TurretToHub extends Command {
    private static final double ACCEL_LIMIT_WHILE_SHOOTING = 1.2 * 0.02;
    private static final double VEL_LIMIT_WHILE_SHOOTING = 0.8;

    private static final double LEAD_TIME_LATENCY_SECONDS = 0.2;
    
    private static final double MAX_ANGLE_ERROR_RADIANS = Degrees.of(5).in(Radians);
    private static final double MAX_RPM_ERROR = 30;

    private final TurretSubsystem shooter;
    private final SwerveSubsystem swerve;
    private final SwerveInputStream swerveInputFieldOriented;
    private final IndexerSubsystem indexer;

    private final LEDSubsystem led;

    private ChassisSpeeds lastSwerveSpeeds = new ChassisSpeeds();

    private ProfiledPIDController turnController = new ProfiledPIDController(10, 0, 0,
            new TrapezoidProfile.Constraints(10, 3));

    private boolean isSpunUp = false;

    public TurretToHub(TurretSubsystem shooter, SwerveSubsystem swerve, IndexerSubsystem indexer,
            SwerveInputStream swerveInputFieldOriented, LEDSubsystem led) {
        this.shooter = shooter;
        this.swerve = swerve;
        this.swerveInputFieldOriented = swerveInputFieldOriented;
        this.indexer = indexer;
        this.led = led;

        led = new LEDSubsystem();

        turnController.enableContinuousInput(-Math.PI, Math.PI);

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
        swerveSpeeds = SwerveSubsystem.applyAccelLimit(lastSwerveSpeeds, swerveSpeeds, ACCEL_LIMIT_WHILE_SHOOTING);
        swerveSpeeds = SwerveSubsystem.applyVelocityLimit(swerveSpeeds, VEL_LIMIT_WHILE_SHOOTING);
        lastSwerveSpeeds = swerveSpeeds;

        Pose2d turretPose = shooter.getTurretPoseFromRobotPose(swerve.getPose());

        Strategy.StrategyConfig strategyConfig = Strategy.getLocationTarget(swerve.getPose().getTranslation());
        Translation2d locationTarget = strategyConfig.targetLocation;
        Translation2d commandedRobotVelocity = new Translation2d(swerveSpeeds.vxMetersPerSecond,
                swerveSpeeds.vyMetersPerSecond);

        // Calculate the lead time we need based on distance
        // adjust the constant based on system latency
        Distance distanceToLocationTarget = Meters.of(locationTarget.getDistance(turretPose.getTranslation()));
        double leadTimeSeconds = LEAD_TIME_LATENCY_SECONDS + shooter.timeUntilHit(distanceToLocationTarget.in(Feet)).in(Seconds);

        // Lead the target based on our current velocity
        locationTarget = locationTarget.minus(commandedRobotVelocity.times(leadTimeSeconds));
        distanceToLocationTarget = Meters.of(locationTarget.getDistance(turretPose.getTranslation()));

        SmartDashboard.putNumber("TurretToHub/distanceToTarget", distanceToLocationTarget.in(Feet));

        AngularVelocity targetShooterSpeed = shooter.targetShooterSpeed(distanceToLocationTarget.in(Feet));
        // AngularVelocity targetShooterSpeed = RPM.of(SmartDashboard.getNumber("Target RPM", 2000));
        shooter.setShooterSpeed(targetShooterSpeed);
        SmartDashboard.putNumber("TurretToHub/shooterSpeed", targetShooterSpeed.in(RPM));

        if (strategyConfig.strategyType == StrategyType.HUB_SHOT) {
            shooter.setHoodAngle(shooter.targetHoodAngle(distanceToLocationTarget.in(Feet)));
            SmartDashboard.putNumber("TurretToHub/hoodAngle", shooter.targetHoodAngle(distanceToLocationTarget.in(Feet)).in(Degrees));
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
            //LED blue = shooting
            led.setLEDState(LEDState.BLUE);
        } else {
            indexer.stopIndexing();
            //LED green = ready to shoot
            led.setLEDState(LEDState.GREEN);

        }
    }
}
