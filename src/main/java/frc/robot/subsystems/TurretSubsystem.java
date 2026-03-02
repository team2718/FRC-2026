package frc.robot.subsystems;

import static edu.wpi.first.units.Units.Degree;
import static edu.wpi.first.units.Units.Degrees;
import static edu.wpi.first.units.Units.Inches;
import static edu.wpi.first.units.Units.Meters;
import static edu.wpi.first.units.Units.RPM;
import static edu.wpi.first.units.Units.Seconds;

import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.Follower;
import com.ctre.phoenix6.controls.MotionMagicVelocityVoltage;
import com.ctre.phoenix6.controls.NeutralOut;
import com.ctre.phoenix6.controls.PositionVoltage;
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

    private final static Angle hoodMinAngle = Degrees.of(80); // TODO: Set to the "true" angle when at the

    private final static Distance turretX = Inches.of(-5.75);
    private final static Distance turretY = Inches.of(-5);
    private final static Angle turretAngle = Degrees.of(-85);
    private final static Transform2d turretLocation = new Transform2d(new Translation2d(turretX.in(Meters), turretY.in(Meters)), Rotation2d.fromDegrees(turretAngle.in(Degrees)));

    // private double turretDistanceToRobotCenter = 0.5;
    // private double turretDegreeFromRobotCenter = 40;

    // double robotAngleFromTag9 =
    // hubCenterLocation.minus(swerve.getPose().getTranslation()).getAngle().getDegrees();
    // double robotDistanceToTag9 = Math.sqrt(Math.pow(swerve.getPose().getX() -
    // hubCenterLocation.getX(), 2)
    // + Math.pow(swerve.getPose().getY() - hubCenterLocation.getY(), 2));

    // double turretPositionX;
    // double turretPositionY;

    // double turretAngleFromTag9;
    // double turretDistanceToTag9;

    // double projectedTime = Math.sqrt(robotDistanceToTag9);

    // double projectedTurretPositionX;
    // double projectedTurretPositionY;

    // double projectedTurretAngleFromTag9;
    // double projectedTurretDistanceToTag9;

    private boolean turretEnabled = true;

    public TurretSubsystem() {

        // Motors-----------------------------------------------------------------------------------------------------------------------------------

        //Tthe left turret shooter is the primary motor and the right shooter follows it

        turretshooterLeft = new TalonFX(Constants.TurretConstants.LEFT_SHOOTER_MOTOR_ID);
        turretshooterRight = new TalonFX(Constants.TurretConstants.RIGHT_SHOOTER_MOTOR_ID);
        turrethood = new TalonFX(Constants.TurretConstants.HOOD_MOTOR_ID);

        // Configuring motor variables (The current limit is set to 5 amps for now)

        TalonFXConfiguration turretshooterconfig = new TalonFXConfiguration();
        turretshooterconfig.CurrentLimits.StatorCurrentLimit = 5; // TODO: increase back to 25
        turretshooterconfig.MotorOutput.NeutralMode = NeutralModeValue.Coast;

        turretshooterconfig.Slot0.kG = 0.0;
        turretshooterconfig.Slot0.kS = 0.0;
        // kV is is in V/rps. kV given is in RPM/V.
        // (1 / (RPM/V)) gives us V/RPM, and multiplying by 60 gives us V/rps.
        // 485.6 is kV of Kraken x60 in FOC. Use 500.0 for non-FOC.
        // See https://www.reca.lc/motors
        turretshooterconfig.Slot0.kV = 1.0 / 500.0 * 60.0;
        turretshooterconfig.Slot0.kA = 0.19; // TODO: Tune, this value is from reca.lc

        turretshooterconfig.Slot0.kP = 0.1;
        turretshooterconfig.Slot0.kI = 0.0;
        turretshooterconfig.Slot0.kD = 0.0;
        turretshooterconfig.MotionMagic.MotionMagicCruiseVelocity = 0;
        turretshooterconfig.MotionMagic.MotionMagicAcceleration = 2000;
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

        turretshooterRight.setControl(new Follower(Constants.TurretConstants.LEFT_SHOOTER_MOTOR_ID, MotorAlignmentValue.Opposed));
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

        turretshooterLeft.setControl(new MotionMagicVelocityVoltage(angularVelocity));
    }

    // sets speed of the shooter
    public void setShooterSpeedRPM(double rpm) {
        setShooterSpeed(RPM.of(rpm));
    }

    public void stopShooter() {
        // Should we do this or should we use stopMotor() on each motor?
        // Using the closed loop gives us smoother deceleration
        setShooterSpeedRPM(0);
    }

    // Return the average RPM of the two shooter motors (should be the same, but
    // just in case)
    public double getShooterRPM() {
        return turretshooterLeft.getVelocity().getValue().in(RPM);
    }

    public boolean shooterAtSpeed(double tolerance) {
        double targetRPM = turretshooterLeft.getClosedLoopReference().getValue() * 60.0;

        if (targetRPM < 100) {
            return false;
        }

        double currentRPM = getShooterRPM();
        // SmartDashboard.putNumber("Shooter At Speed Target RPM", targetRPM);
        // SmartDashboard.putNumber("Shooter At Speed Current RPM", currentRPM);
        return Math.abs(currentRPM - targetRPM) < tolerance;
    }

    // // sets rotational speed of the turret
    // public void setTurretPosition(double power) {
    // turretspinner.set(power);
    // }

    // // returns the current rotational position of the turret
    // public double getTurretPosition() {
    // return turretspinner.getEncoder().getPosition();
    // }

    // sets rotational speed of the hood
    public void setHoodAngle(Angle angle) {
        if (!turretEnabled) {
            return;
        }

        // Clamp angle from 45 degrees to 80 degrees
        if (angle.in(Degrees) < 45) {
            angle = Degrees.of(45);
        } else if (angle.in(Degrees) > 80) {
            angle = Degrees.of(80);
        }

        // The hood is at position 0 at the bottom.
        // So a target angle of 45 degrees would be 45 - hoodMinAngle
        // where hoodMinAngle is the angle of the hood when it's at position 0
        SmartDashboard.putNumber("Hood Target Position", hoodMinAngle.minus(angle).times(50.57).in(Degrees));
        turrethood.setControl(new PositionVoltage(hoodMinAngle.minus(angle).times(50.57)));

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
    public Angle getTurretHood() {
        return turrethood.getPosition().getValue().minus(hoodMinAngle);
    }

    // Estimates the angle we want to shoot the fuel at based on the turret's
    // distance to the hub (20 = angle the ball shoots at when the hood is at 80
    // degrees, not sure what is actually is)
    // The numbers used here arised from tinkering around to get an accurate
    // estimate equasion (28500 / ("Distance To Hub" + 25) ^ 2) + 46.25
    public Angle targetHoodAngle(double distance) {
        return Degree.of(107 * Math.pow(distance, -0.228));
    }

    // Estimates the speed we want to shoot the fuel at based on the turret's
    // distance to the hub
    public AngularVelocity targetShooterSpeed(double distanceFeet) {
        double velocityFtps = 17.5 + 0.54 * distanceFeet;

        // Adjust the below numbers based on testing.
        // The 110 is a conversion factor to convert from ft/s to RPM, and the 0.0 is just a constant to adjust the speed up or down.
        return RPM.of(velocityFtps * 110 + 0.0);
    }

    public Time timeUntilHit(double distanceFeet) {
        return Seconds.of(distanceFeet * 0.02 + 0.8);
    }

    // public void setHoodToAngle(double angle) {

    // if (Math.abs(getWrappedAngleDifference(getTurretHood(), angle)) < 0.3) {
    // setTurretHood(0);
    // return;
    // }

    // setTurretHood(1 * getWrappedAngleDifference(getTurretHood(), angle));

    // }

    @Override
    public void periodic() {
        if (!turretEnabled) {
            turrethood.set(0);
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