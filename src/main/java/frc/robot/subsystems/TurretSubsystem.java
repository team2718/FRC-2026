package frc.robot.subsystems;

import static edu.wpi.first.units.Units.Degree;
import static edu.wpi.first.units.Units.Degrees;
import static edu.wpi.first.units.Units.Inches;
import static edu.wpi.first.units.Units.Meters;
import static edu.wpi.first.units.Units.RPM;
import static edu.wpi.first.units.Units.Rotations;
import static edu.wpi.first.units.Units.Seconds;

import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.Follower;
import com.ctre.phoenix6.controls.MotionMagicVoltage;
import com.ctre.phoenix6.controls.NeutralOut;
import com.ctre.phoenix6.controls.PositionVoltage;
import com.ctre.phoenix6.controls.VelocityVoltage;
import com.ctre.phoenix6.hardware.TalonFX;
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
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
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
    @Logged(name = "Turret Azimuth Motor")
    private final TalonFX turretspinnyspinner;

    private final static AngularVelocity MAX_FLYWHEEL_SPEED = RPM.of(5200); // KrakenX60 Efficiency tanks after this

    private final static Angle hoodZeroAngle = Degrees.of(70);
    private final static Angle hoodRangeOfMotion = Degrees.of(30);
    private final static double hoodGearRatio = 286.0/22.0 * 30.0/12.0;

    private final static double turretZeroAngleDegreesDefault = 41.5;
    private static double turretZeroAngleDegrees = turretZeroAngleDegreesDefault;
    private final static double turretRangeOfMotionDegrees = 350; // should be 350 :)
    private final static double turretGearRatio = 160.0/16.0 * 30.0/16.0;

    private final static Distance turretX = Inches.of(-5.692);
    private final static Distance turretY = Inches.of(-5);
    private final static Transform2d turretLocation = new Transform2d(new Translation2d(turretX.in(Meters), turretY.in(Meters)), Rotation2d.fromDegrees(turretZeroAngleDegrees));

    private static final double[][] DISTANCE_TABLE = {
            // dist_ft hood_deg rpm flight_sec
            { 5.0, 70.0, 3300.0, 1.00 }, // measured
            { 7.0, 63.5, 3450.0, 0.90 }, // measured
            { 9.0, 61.0, 3700.0, 0.90 }, // measured
            { 11.0, 59.0, 3900.0, 0.95 }, // measured
            { 13.0, 55.0, 4150.0, 0.98 }, // measured
            { 15.0, 52.0, 4350.0, 1.05 }, // measured
            { 17.0, 50.0, 4650.0, 1.10 }, // measured
    };

    // InterpolatingDoubleTreeMap performs linear interpolation to give us values
    // for distances that aren't in the shot table
    private static final InterpolatingDoubleTreeMap hoodAngleMap = new InterpolatingDoubleTreeMap();
    private static final InterpolatingDoubleTreeMap shooterSpeedMap = new InterpolatingDoubleTreeMap();
    private static final InterpolatingDoubleTreeMap flightTimeMap = new InterpolatingDoubleTreeMap();

    static {
        for (double[] row : DISTANCE_TABLE) {
            double distFt = row[0];
            hoodAngleMap.put(distFt, row[1]);
            shooterSpeedMap.put(distFt, row[2]);
            flightTimeMap.put(distFt, row[3]);
        }
    }

    private boolean turretEnabled = true;

    TalonFXConfiguration turretshooterconfig;
    TalonFXConfiguration turrethoodconfig;
    TalonFXConfiguration turretAzimuthConfiguration;

    private double targetAngleDegrees = 0;

    public TurretSubsystem() {

        // Motors-----------------------------------------------------------------------------------------------------------------------------------

        //Tthe left turret shooter is the primary motor and the right shooter follows it

        turretshooterLeft = new TalonFX(Constants.TurretConstants.LEFT_SHOOTER_MOTOR_ID);
        turretshooterRight = new TalonFX(Constants.TurretConstants.RIGHT_SHOOTER_MOTOR_ID);
        turrethood = new TalonFX(Constants.TurretConstants.HOOD_MOTOR_ID);
        turretspinnyspinner = new TalonFX(Constants.TurretConstants.turretSpinnerID);

        // Configuring motor variables (The current limit is set to 5 amps for now)

        turretshooterconfig = new TalonFXConfiguration();
        turretshooterconfig.CurrentLimits.StatorCurrentLimit = 80;
        turretshooterconfig.CurrentLimits.SupplyCurrentLimit = 60;
        turretshooterconfig.MotorOutput.NeutralMode = NeutralModeValue.Coast;

        turretshooterconfig.Slot0.kS = 0.1;
        turretshooterconfig.Slot0.kV = 0.115;
        turretshooterconfig.Slot0.kP = 0.10;
        
        turretshooterconfig.Slot0.kI = 0.0;
        turretshooterconfig.Slot0.kD = 0.0;
        
        turretshooterconfig.MotorOutput.Inverted = InvertedValue.Clockwise_Positive;
        turretshooterRight.getConfigurator().apply(turretshooterconfig);

        turretshooterconfig.MotorOutput.Inverted = InvertedValue.CounterClockwise_Positive;
        turretshooterLeft.getConfigurator().apply(turretshooterconfig);

        turrethoodconfig = new TalonFXConfiguration();

        turrethoodconfig.CurrentLimits.StatorCurrentLimit = 10;
        // turrethoodconfig.OpenLoopRamps.VoltageOpenLoopRampPeriod = 0.3;
        turrethoodconfig.MotorOutput.NeutralMode = NeutralModeValue.Brake;
        turrethoodconfig.MotorOutput.Inverted = InvertedValue.Clockwise_Positive;
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

        turretAzimuthConfiguration = new TalonFXConfiguration();
        turretAzimuthConfiguration.MotorOutput.NeutralMode = NeutralModeValue.Brake;
        turretAzimuthConfiguration.MotorOutput.Inverted = InvertedValue.Clockwise_Positive;
        turretAzimuthConfiguration.MotorOutput.PeakForwardDutyCycle = 0.35;
        turretAzimuthConfiguration.MotorOutput.PeakReverseDutyCycle = -0.35;
        turretAzimuthConfiguration.CurrentLimits.StatorCurrentLimit = 30;
        turretAzimuthConfiguration.CurrentLimits.SupplyCurrentLimit = 40;
        turretAzimuthConfiguration.Slot0.kP = 28.0;
        turretAzimuthConfiguration.Slot0.kS = 0.45;
        turretAzimuthConfiguration.Slot0.kV = 0.122;
        turretAzimuthConfiguration.MotionMagic.MotionMagicCruiseVelocity = 25;
        turretAzimuthConfiguration.MotionMagic.MotionMagicAcceleration = 150;
        turretAzimuthConfiguration.MotionMagic.MotionMagicJerk = 1000;
        // turretAzimuthConfiguration.SoftwareLimitSwitch.ForwardSoftLimitEnable = true;
        turretAzimuthConfiguration.SoftwareLimitSwitch.ReverseSoftLimitEnable = true;
        // turretAzimuthConfiguration.SoftwareLimitSwitch.ForwardSoftLimitThreshold = (225.0/360.0) * turretGearRatio - 0.1;
        turretAzimuthConfiguration.SoftwareLimitSwitch.ReverseSoftLimitThreshold = 0.1;
        turretspinnyspinner.getConfigurator().apply(turretAzimuthConfiguration);

        turretspinnyspinner.setPosition(0);

        // Make sure frequency is high enough for follower to follow
        turretshooterLeft.getMotorVoltage().setUpdateFrequency(100);
        turretshooterRight.setControl(new Follower(Constants.TurretConstants.LEFT_SHOOTER_MOTOR_ID, MotorAlignmentValue.Opposed));
    }

    public void setShooterSpeed(AngularVelocity angularVelocity) {
        if (!turretEnabled) {
            return;
        }

        angularVelocity = applyFlywheelSpeedBounds(angularVelocity);

        turretshooterLeft.setControl(new VelocityVoltage(angularVelocity));
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
        return Math.abs(currentRPM - (targetRPM)) < tolerance;
    }

    public void adjustLeft() {
        turretZeroAngleDegrees -= 2;
    }

    public void adjustRight() {
        turretZeroAngleDegrees += 2;
    }

    public void adjustReset() {
        turretZeroAngleDegrees = turretZeroAngleDegreesDefault;
    }

    // Sets position of the turret in degrees
    // 0 is facing forward (towards the front of the robot)
    // CW is positive, CCW is negative
    // return true if possible to go to this angle
    public boolean setAzimuthAngle(Angle targetAngle) {
        if (!turretEnabled) {
            return false; // Return early so we don't shoot when the turret is disabled
        }

        targetAngleDegrees = targetAngle.in(Degrees) - turretZeroAngleDegrees;

        // Make sure the turret angle is [0 360)
        if (targetAngleDegrees < 0) {
            targetAngleDegrees += 360;
        } else if (targetAngleDegrees >= 360) {
            targetAngleDegrees -= 360;
        }

        // If the target angle is outside the range of motion, clamp it to the nearest valid angle
        if (targetAngleDegrees < 0) {
            targetAngleDegrees = 0;
            turretspinnyspinner.setControl(new MotionMagicVoltage(targetAngleDegrees * turretGearRatio / 360.0));
            return false;
        } else if (targetAngleDegrees > turretRangeOfMotionDegrees) {
            targetAngleDegrees = turretRangeOfMotionDegrees;
            turretspinnyspinner.setControl(new MotionMagicVoltage(targetAngleDegrees * turretGearRatio / 360.0));
            return false;
        }

        // Convert target angle in degrees to motor rotations and set the motor to that position using Motion Magic
        // Subtract turretZeroAngleDegrees because the motor position is 0 at turretZeroAngleDegrees, not at 0 degrees
        turretspinnyspinner.setControl(new MotionMagicVoltage(targetAngleDegrees * turretGearRatio / 360.0));
        return true;
    }

    public void setTurretSpeedUnchecked(double power) {
        if (!turretEnabled) {
            return;
        }

        turretspinnyspinner.set(power);
    }

    public void stopTurret() {
        turretspinnyspinner.setControl(new NeutralOut());
    }

    public void setTurretNoLimits() {
        // turretAzimuthConfiguration.SoftwareLimitSwitch.ForwardSoftLimitEnable = false;
        turretAzimuthConfiguration.SoftwareLimitSwitch.ReverseSoftLimitEnable = false;
        turretspinnyspinner.getConfigurator().apply(turretAzimuthConfiguration);
    }

    public void setTurretYesLimits() {
        // turretAzimuthConfiguration.SoftwareLimitSwitch.ForwardSoftLimitEnable = true;
        turretAzimuthConfiguration.SoftwareLimitSwitch.ReverseSoftLimitEnable = true;
        turretspinnyspinner.getConfigurator().apply(turretAzimuthConfiguration);
    }

    public Current getTurretSpinnerCurrent() {
        return turretspinnyspinner.getStatorCurrent().getValue();
    }

    public void resetTurretPosition() {
        turretspinnyspinner.setPosition(0);
        System.out.println("Turret position reset!");
    }

    // returns the current position of the turret
    // where 0 degrees is facing forward, positive is CW, negative is CCW
    public Angle getTurretAngle() {
        return Degrees.of(turretZeroAngleDegrees + (turretspinnyspinner.getPosition().getValue().in(Degrees) / turretGearRatio));
    }

    public double getTurretAngleDegrees() {
        return getTurretAngle().in(Degrees);
    }


    //Sets the target turret angle based on where the turret is projected to be on the field when the fuel hits the hub
    public Angle targetTurretAngle(double targetAngle) {
        return Degree.of(targetAngle);
    }

    // Estimates the angle we want to shoot the fuel at based on the turret's
    // distance to the hub (20 = angle the ball shoots at when the hood is at 80
    // degrees, not sure what is actually is)
    // The numbers used here arised from tinkering around to get an accurate
    // estimate equasion (28500 / ("Distance To Hub" + 25) ^ 2) + 46.25
    public Angle targetHoodAngle(double distanceFeet) {
        return Degree.of(hoodAngleMap.get(distanceFeet));
    }

    // Estimates the speed we want to shoot the fuel at based on the turret's
    // distance to the hub
    public AngularVelocity targetShooterSpeed(double distanceFeet) {
        return RPM.of(shooterSpeedMap.get(distanceFeet));
    }

    // sets position of the hood
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
        turrethood.setControl(new NeutralOut());
    }

    public void dropHood() {
        if (!turretEnabled) {
            return;
        }

        // Drop the hood down to the zero position (lowest angle)
        turrethood.setControl(new PositionVoltage(0));
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
        return hoodZeroAngle.minus(Rotations.of(turrethood.getPosition().getValueAsDouble() / hoodGearRatio)).in(Degrees);
    }

    // Gets the time it takes for the fuel to reach the hub based of distance
    public Time timeUntilHit(double distanceFeet) {
        /*return Seconds.of(distanceFeet * 0.02 + 0.8);
        return Seconds.of(distanceFeet / Math.cos(getTurretHoodAngleDegrees()) * getShooterRPM()); */
        
        return Seconds.of(flightTimeMap.get(distanceFeet) // * [Current Shooter RPM] / [Shooter RPM used in these measurements]
        
        );
    }

    @Override
    public void periodic() {
        if (!turretEnabled) {
            turrethood.setControl(new NeutralOut());
            turretshooterLeft.setControl(new NeutralOut());
        }

        SmartDashboard.putNumber("Turret Offset", turretZeroAngleDegrees);
    }

    public void setEnabled(boolean turretEnabled) {
        this.turretEnabled = turretEnabled;
    }

    public Pose2d getTurretPoseFromRobotPose(Pose2d robotPose) {
        return robotPose.plus(turretLocation);
    }

    public double getTargetPositionRotations() {
        return targetAngleDegrees;
    }

    public AngularVelocity applyFlywheelSpeedBounds(AngularVelocity targetShooterSpeed) {
        if (targetShooterSpeed.lt(RPM.zero())) {
            return RPM.zero();
        } else if (targetShooterSpeed.gt(MAX_FLYWHEEL_SPEED)) {
            return MAX_FLYWHEEL_SPEED;
        } else {
            return targetShooterSpeed;
        }
    }

}