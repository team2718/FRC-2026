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
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.networktables.StructPublisher;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.units.measure.Distance;
import edu.wpi.first.wpilibj.Timer;
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
import frc.robot.utils.PossumUtils;
import swervelib.SwerveInputStream;

public class TurretToHubCalibration extends Command {

    // Setting variables for everything (constants, subsystems, motors)

    private static final double ACCEL_LIMIT_WHILE_SHOOTING = 1.4 * 0.02;
    private static final double VEL_LIMIT_WHILE_SHOOTING = 1.0;

    private static final double ROT_ACCEL_LIMIT_WHILE_SHOOTING = 1.3 * 0.02;
    private static final double ROT_VEL_LIMIT_WHILE_SHOOTING = 1.3;

    private static final double LEAD_TIME_LATENCY_SECONDS = 0.22; // translation lead time
    private static final double HEADING_LEAD_TIME_SECONDS = 0.15; // rotation lead time

    private static final double MAX_RPM_ERROR = 50;

    private final TurretSubsystem shooter;
    private final SwerveSubsystem swerve;
    private final SwerveInputStream swerveInputFieldOriented;
    private final IndexerSubsystem indexer;
    private final IntakeSubsystem intake;

    private final LEDSubsystem led;

    private ChassisSpeeds lastSwerveSpeeds = new ChassisSpeeds();

    private boolean isSpunUp = false;

    private final double redTrenchX = 11.916; // hard-coded yippee (in meters from blue driver station wall)
    private final double blueTrenchX = 4.626;

    private Pose2d turretPose;

    private double startTime = Double.MAX_VALUE;

    StructPublisher<Translation2d> targetPublisher = NetworkTableInstance.getDefault()
            .getStructTopic("TurretToHub/TargetTranslation", Translation2d.struct).publish();
    StructPublisher<Translation2d> ledTargetPublisher = NetworkTableInstance.getDefault()
            .getStructTopic("TurretToHub/LedTargetTranslation", Translation2d.struct).publish();

    public TurretToHubCalibration(TurretSubsystem shooter, SwerveSubsystem swerve, IndexerSubsystem indexer,
            IntakeSubsystem intake,
            SwerveInputStream swerveInputFieldOriented, LEDSubsystem led) {
        this.shooter = shooter;
        this.swerve = swerve;
        this.indexer = indexer;
        this.intake = intake;
        this.swerveInputFieldOriented = swerveInputFieldOriented;
        this.led = led;

        SmartDashboard.putNumber("Calibration/HoodAngle", 70);
        SmartDashboard.putNumber("Calibration/RPM", 3000);

        addRequirements(shooter, swerve, indexer, intake, led);
    }

    // Resets variables when the program starts
    @Override
    public void initialize() {
        // Run intake while shooting
        intake.setIntakeSpeed(0.75);

        startTime = Timer.getFPGATimestamp();
    }

    // Spins the turret and the hood to their respective target positions when
    // activated
    @Override
    public void execute() {
        double hoodAngle = SmartDashboard.getNumber("Calibration/HoodAngle", 70);
        double rpm = SmartDashboard.getNumber("Calibration/RPM", 3000);

        turretPose = shooter.getTurretPoseFromRobotPose(swerve.getPose());
        Strategy.StrategyConfig strategyConfig = Strategy.getLocationTarget(turretPose.getTranslation());
        Distance distanceToLocationTarget = Meters.of(strategyConfig.targetLocation.getDistance(turretPose.getTranslation()));
        SmartDashboard.putNumber("TurretToHub/Distance to Target Feet", distanceToLocationTarget.in(Feet));

        shooter.setShooterSpeedRPM(rpm);
        shooter.setHoodAngle(Degrees.of(hoodAngle));

        if (Timer.getFPGATimestamp() - startTime > 2) {
            indexer.runIndexing(RPM.of(rpm));
            led.setLEDState(LEDState.GREEN);
        } else {
            led.setLEDState(LEDState.YELLOW);
        }
    }

    // When the program ends, stop everything
    @Override
    public void end(boolean interuppted) {

        intake.setIntakeSpeed(0);
        indexer.stopIndexing();

        led.setLEDState(LEDState.TEAL);

        shooter.dropHood();
        shooter.stopShooter();
    }

    // YOUR JOB WILL NEVER BE FINISHED
    @Override
    public boolean isFinished() {
        return false;
    }
}
