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

    private final static Angle hoodZeroAngle = Degrees.of(70);
    private final static Angle hoodRangeOfMotion = Degrees.of(30);
    private final static double hoodGearRatio = 286.0/22.0 * 30.0/12.0;

    private final static double turretZeroAngleDegrees = 52;
    private final static double turretRangeOfMotionDegrees = 330;
    private final static double turretGearRatio = 160.0/16.0 * 30.0/16.0;

    private final static Distance turretX = Inches.of(-5.75);
    private final static Distance turretY = Inches.of(-5);
    private final static Transform2d turretLocation = new Transform2d(new Translation2d(turretX.in(Meters), turretY.in(Meters)), Rotation2d.fromDegrees(turretZeroAngleDegrees));

    private static final double[][] DISTANCE_TABLE = {
            // dist_ft hood_deg rpm flight_sec
            { 4.0, 73.5, 1880.0, 0.88 },
            { 6.0, 69.0, 2000.0, 0.92 },
            { 8.0, 65.0, 2100.0, 0.96 },
            { 10.0, 62.0, 2290.0, 1.00 },
            { 12.0, 59.5, 2350.0, 1.04 },
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

        turretshooterconfig.Slot0.kS = 0.2;
        turretshooterconfig.Slot0.kV = 0.115;
        turretshooterconfig.Slot0.kP = 0.3;
        
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
        turretAzimuthConfiguration.MotorOutput.PeakForwardDutyCycle = 0.3;
        turretAzimuthConfiguration.MotorOutput.PeakReverseDutyCycle = -0.3;
        turretAzimuthConfiguration.CurrentLimits.StatorCurrentLimit = 25;
        turretAzimuthConfiguration.CurrentLimits.SupplyCurrentLimit = 40;
        turretAzimuthConfiguration.Slot0.kP = 20.0;
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

        // Clamp from 0 to 5000 RPM
        if (angularVelocity.in(RPM) < 0) {
            angularVelocity = RPM.of(0);
        } else if (angularVelocity.in(RPM) > 5000) {
            angularVelocity = RPM.of(5000);
        }

        // targetRPM = angularVelocity.in(RPM);

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

    // Sets position of the turret in degrees
    // 0 is facing forward (towards the front of the robot)
    // CW is positive, CCW is negative
    public double setAzimuthAngle(Angle targetAngle) {
        if (!turretEnabled) {
            return 1000; // Return a large error so we don't shoot when the turret is disabled
        }

        targetAngleDegrees = targetAngle.in(Degrees);

        // Convert target angle to the range [0, 360)
        targetAngleDegrees = ((targetAngleDegrees % 360) + 360) % 360;

        SmartDashboard.putNumber("Turret/Target Angle", targetAngleDegrees);
        SmartDashboard.putNumber("Turret/Max Range", turretZeroAngleDegrees + turretRangeOfMotionDegrees);

        // If the target angle is outside the range of motion, clamp it to the nearest valid angle
        // Smallest value is turretZeroAngle, largest is turretZeroAngle + turretRangeOfMotion
        if (targetAngleDegrees < turretZeroAngleDegrees) {
            return 1000; // Return a large error so we don't shoot when the target angle is out of bounds
        } else if (targetAngleDegrees > turretZeroAngleDegrees + turretRangeOfMotionDegrees) {
            return 1000; // Return a large error so we don't shoot when the target angle is out of bounds
        }

        // Convert target angle in degrees to motor rotations and set the motor to that position using Motion Magic
        // Subtract turretZeroAngleDegrees because the motor position is 0 at turretZeroAngleDegrees, not at 0 degrees
        turretspinnyspinner.setControl(new MotionMagicVoltage((targetAngleDegrees - turretZeroAngleDegrees) * turretGearRatio / 360.0));

        // Return the error in degrees (for use in commands)
        return getTurretAngleDegrees() - targetAngleDegrees;

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
    public double getTurretAngleDegrees() {
        return turretspinnyspinner.getPosition().getValueAsDouble() / turretGearRatio + turretZeroAngleDegrees;
        // return turretZeroAngle.minus(Rotations.of(turretspinnyspinner.getPosition().getValueAsDouble() / turretGearRatio)).in(Degrees);
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
        return RPM.of(shooterSpeedMap.get(distanceFeet) * 1.7);
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

}