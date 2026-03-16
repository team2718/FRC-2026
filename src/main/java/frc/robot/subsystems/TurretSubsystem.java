package frc.robot.subsystems;

import static edu.wpi.first.units.Units.Degrees;
import static edu.wpi.first.units.Units.Inches;
import static edu.wpi.first.units.Units.Meters;
import static edu.wpi.first.units.Units.RPM;
import static edu.wpi.first.units.Units.Rotations;
import static edu.wpi.first.units.Units.Seconds;

import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.Follower;
import com.ctre.phoenix6.controls.NeutralOut;
import com.ctre.phoenix6.controls.PositionVoltage;
import com.ctre.phoenix6.controls.VelocityVoltage;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.GravityTypeValue;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.MotorAlignmentValue;
import com.ctre.phoenix6.signals.NeutralModeValue;

import edu.wpi.first.epilogue.Logged;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Transform2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.interpolation.InterpolatingDoubleTreeMap;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.units.measure.Current;
import edu.wpi.first.units.measure.Distance;
import edu.wpi.first.units.measure.Time;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

@Logged
public class TurretSubsystem extends SubsystemBase {

    @Logged(name = "Left Shooter Motor")
    private final TalonFX turretshooterLeft;
    @Logged(name = "Right Shooter Motor")
    private final TalonFX turretshooterRight;
    @Logged(name = "Hood Motor")
    private final TalonFX turrethood;

    private final static Angle hoodZeroAngle = Degrees.of(86);
    private final static Angle hoodRangeOfMotion = Degrees.of(40);
    private final static double hoodGearRatio = (291.0 / 24.0) * (48.0 / 14.0);

    private final static Distance turretX = Inches.of(-5.75);
    private final static Distance turretY = Inches.of(-5);
    private final static Angle turretAngle = Degrees.of(-85);
    private final static Transform2d turretLocation = new Transform2d(
            new Translation2d(turretX.in(Meters), turretY.in(Meters)), Rotation2d.fromDegrees(turretAngle.in(Degrees)));

    // Determine values empirically by testing
    // hood_deg should not be changed as it was determined from simulation
    // and should always be
    // hood_deg = 107 * Math.pow(dist_ft, -0.228)
    // flight_sec is hard to measure but should be close to
    // flight_sec = dist_ft * 0.02 + 0.8
    private static final double[][] SHOT_TABLE = {
            // dist_ft hood_deg rpm flight_sec
            { 4.0, 73.5, 1880.0, 0.88 },
            { 6.0, 69.0, 2000.0, 0.92 },
            { 8.0, 65.0, 2150.0, 0.96 },
            { 10.0, 62.0, 2290.0, 1.00 },
            { 12.0, 59.5, 2430.0, 1.04 },
            { 14.0, 57.5, 2570.0, 1.08 },
            { 16.0, 56.0, 2710.0, 1.12 },
            { 18.0, 54.5, 2850.0, 1.16 },
            { 20.0, 53.5, 2990.0, 1.20 },
    };

    // InterpolatingDoubleTreeMap performs linear interpolation to give us values
    // for distances that aren't in the shot table
    private static final InterpolatingDoubleTreeMap hoodAngleMap = new InterpolatingDoubleTreeMap();
    private static final InterpolatingDoubleTreeMap shooterSpeedMap = new InterpolatingDoubleTreeMap();
    private static final InterpolatingDoubleTreeMap flightTimeMap = new InterpolatingDoubleTreeMap();

    static {
        for (double[] row : SHOT_TABLE) {
            double distFt = row[0];
            hoodAngleMap.put(distFt, row[1]);
            shooterSpeedMap.put(distFt, row[2]);
            flightTimeMap.put(distFt, row[3]);
        }
    }

    private boolean turretEnabled = true;

    public TurretSubsystem() {

        // Motors-----------------------------------------------------------------------------------------------------------------------------------

        // Tthe left turret shooter is the primary motor and the right shooter follows
        // it

        turretshooterLeft = new TalonFX(Constants.TurretConstants.LEFT_SHOOTER_MOTOR_ID);
        turretshooterRight = new TalonFX(Constants.TurretConstants.RIGHT_SHOOTER_MOTOR_ID);
        turrethood = new TalonFX(Constants.TurretConstants.HOOD_MOTOR_ID);

        // Configuring motor variables (The current limit is set to 5 amps for now)

        TalonFXConfiguration turretshooterconfig = new TalonFXConfiguration();
        turretshooterconfig.CurrentLimits.StatorCurrentLimit = 80;
        turretshooterconfig.CurrentLimits.SupplyCurrentLimit = 60;
        turretshooterconfig.MotorOutput.NeutralMode = NeutralModeValue.Coast;

        turretshooterconfig.Slot0.kG = 0.0;
        turretshooterconfig.Slot0.kS = 0.0;
        // kV is is in V/rps. kV given is in RPM/V.
        // (1 / (RPM/V)) gives us V/RPM, and multiplying by 60 gives us V/rps.
        // 485.6 is kV of Kraken x60 in FOC. Use 500.0 for non-FOC.
        // See https://www.reca.lc/motors
        turretshooterconfig.Slot0.kV = 1.0 / 500.0 * 60.0;
        turretshooterconfig.Slot0.kA = 0.19; // TODO: Tune, this value is from reca.lc

        turretshooterconfig.Slot0.kP = 0.4;
        turretshooterconfig.Slot0.kI = 0.0;
        turretshooterconfig.Slot0.kD = 0.0;
        // turretshooterconfig.MotionMagic.MotionMagicCruiseVelocity = 0;
        // turretshooterconfig.MotionMagic.MotionMagicAcceleration = 2000;
        turretshooterconfig.Slot0.GravityType = GravityTypeValue.Elevator_Static;

        turretshooterconfig.MotorOutput.Inverted = InvertedValue.CounterClockwise_Positive;
        turretshooterLeft.getConfigurator().apply(turretshooterconfig);

        turretshooterconfig.MotorOutput.Inverted = InvertedValue.Clockwise_Positive;
        turretshooterRight.getConfigurator().apply(turretshooterconfig);

        TalonFXConfiguration turrethoodconfig = new TalonFXConfiguration();

        turrethoodconfig.CurrentLimits.StatorCurrentLimit = 10;
        // turrethoodconfig.OpenLoopRamps.VoltageOpenLoopRampPeriod = 0.3;
        turrethoodconfig.MotorOutput.NeutralMode = NeutralModeValue.Brake;
        turrethoodconfig.MotorOutput.Inverted = InvertedValue.CounterClockwise_Positive;
        turrethoodconfig.Slot0.kG = 0.0;
        turrethoodconfig.Slot0.kS = 0.0;
        // turrethoodconfig.Slot0.kV = 1.0 / 620.0 * 60.0;
        turrethoodconfig.Slot0.kA = 0.0;
        turrethoodconfig.Slot0.kP = 3.0;
        turrethoodconfig.Slot0.kI = 0.0;
        turrethoodconfig.Slot0.kD = 0.0;
        turrethoodconfig.MotionMagic.MotionMagicCruiseVelocity = 0;
        turrethoodconfig.MotionMagic.MotionMagicAcceleration = 500;

        turrethood.getConfigurator().apply(turrethoodconfig);

        // Make sure frequency is high enough for follower to follow
        turretshooterLeft.getMotorVoltage().setUpdateFrequency(100);
        turretshooterRight
                .setControl(new Follower(Constants.TurretConstants.LEFT_SHOOTER_MOTOR_ID, MotorAlignmentValue.Opposed));
    }

    public void setShooterSpeed(AngularVelocity angularVelocity) {
        if (!turretEnabled) {
            return;
        }

        // Clamp from 0 to 5000 RPM
        if (angularVelocity.in(RPM) < 0) {
            angularVelocity = RPM.of(0);
        } else if (angularVelocity.in(RPM) > 5000) {
            angularVelocity = RPM.of(5000);
        }

        turretshooterLeft.setControl(new VelocityVoltage(angularVelocity).withEnableFOC(true));
    }

    // sets speed of the shooter
    public void setShooterSpeedRPM(double rpm) {
        setShooterSpeed(RPM.of(rpm));
    }

    public void stopShooter() {
        // Should we do this or should we use stopMotor() on each motor?
        // Using the closed loop gives us smoother deceleration
        turretshooterLeft.setControl(new NeutralOut());
    }

    // Return the average RPM of the two shooter motors (should be the same, but
    // just in case)
    public double getShooterRPM() {
        return turretshooterLeft.getVelocity().getValue().in(RPM);
    }

    public boolean shooterAtSpeed(double targetRPM, double tolerance) {
        if (targetRPM < 200) {
            return false;
        }

        double currentRPM = getShooterRPM();
        return Math.abs(currentRPM - targetRPM) < tolerance;
    }

    // sets rotational speed of the hood
    public void setHoodAngle(Angle angle) {
        if (!turretEnabled) {
            return;
        }

        // Clamp angle from ~42 degrees to ~82 degrees
        if (angle.in(Degrees) < hoodZeroAngle.minus(hoodRangeOfMotion).in(Degrees)) {
            angle = hoodZeroAngle.minus(hoodRangeOfMotion);
        } else if (angle.in(Degrees) > hoodZeroAngle.in(Degrees)) {
            angle = hoodZeroAngle;
        }

        // The hood is at position 0 at the bottom.
        // So a target angle of 45 degrees would be hoodMinAngle - 45
        // where hoodMinAngle is the angle of the hood when it's at position 0
        turrethood.setControl(new PositionVoltage(hoodZeroAngle.minus(angle).times(hoodGearRatio)));

        return;

    }

    public void setHoodSpeedUnchecked(double power) {
        if (!turretEnabled) {
            return;
        }

        turrethood.set(power);
    }

    public void stopHood() {
        turrethood.stopMotor();
    }

    public Current getTurretHoodCurrent() {
        return turrethood.getStatorCurrent().getValue();
    }

    public void resetHoodPosition() {
        turrethood.setPosition(0);
        System.out.println("Hood position reset!");
    }

    // returns the current position of the hood
    public double getTurretHoodAngleDegrees() {
        return hoodZeroAngle.minus(Rotations.of(turrethood.getPosition().getValueAsDouble() / hoodGearRatio))
                .in(Degrees);
    }

    // Use the shot table to get the target hood angle using interpolation based on the distance to the target
    public Angle targetHoodAngle(double distanceFeet) {
        return Degrees.of(hoodAngleMap.get(distanceFeet));
    }

    // Use the shot table to get the target shooter speed using interpolation based on the distance to the target
    public AngularVelocity targetShooterSpeed(double distanceFeet) {
        return RPM.of(shooterSpeedMap.get(distanceFeet));
    }

    // Use the shot table to get the target flight time using interpolation based on the distance to the target
    public Time timeUntilHit(double distanceFeet) {
        return Seconds.of(flightTimeMap.get(distanceFeet));
    }

    @Override
    public void periodic() {
        if (!turretEnabled) {
            turrethood.setControl(new NeutralOut());
            turretshooterLeft.setControl(new NeutralOut());
        }
    }

    public void setEnabled(boolean turretEnabled) {
        this.turretEnabled = turretEnabled;
    }

    public Pose2d getTurretPoseFromRobotPose(Pose2d robotPose) {
        return robotPose.plus(turretLocation);
    }

}