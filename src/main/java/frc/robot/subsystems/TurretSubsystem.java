package frc.robot.subsystems;

import static edu.wpi.first.units.Units.Amps;
import static edu.wpi.first.units.Units.RPM;

import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.MotionMagicVelocityVoltage;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.GravityTypeValue;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;

import edu.wpi.first.epilogue.Logged;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

@Logged
public class TurretSubsystem extends SubsystemBase {

    private final TalonFX turretshooterLeft;
    private final TalonFX turretshooterRight;
    private final TalonFX turrethood;

    private double turretDistanceToRobotCenter = 0.5;
    private double turretDegreeFromRobotCenter = 40;

    SwerveSubsystem swerve;
    private final Translation2d hubCenterLocation = new Translation2d(11.92, 4.03);

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

        turretshooterLeft = new TalonFX(Constants.TurretConstants.LEFT_SHOOTER_MOTOR_ID);
        // turretspinner = new SparkMax(Constants.TurretConstants.turretspinnerID,
        // SparkLowLevel.MotorType.kBrushless);
        turretshooterRight = new TalonFX(Constants.TurretConstants.RIGHT_SHOOTER_MOTOR_ID);
        turrethood = new TalonFX(Constants.TurretConstants.HOOD_MOTOR_ID);

        // Configuring motor variables (The current limit is set to 5 amps for now)

        TalonFXConfiguration turretshooterconfig = new TalonFXConfiguration();
        turretshooterconfig.CurrentLimits.StatorCurrentLimit = 25;
        turretshooterconfig.MotorOutput.NeutralMode = NeutralModeValue.Coast;

        turretshooterconfig.Slot0.kG = 0.0;
        turretshooterconfig.Slot0.kS = 0.0;
        turretshooterconfig.Slot0.kV = 0.12;
        turretshooterconfig.Slot0.kA = 0.0;

        turretshooterconfig.Slot0.kP = 0.0;
        turretshooterconfig.Slot0.kI = 0.0;
        turretshooterconfig.Slot0.kD = 0.0;
        turretshooterconfig.MotionMagic.MotionMagicCruiseVelocity = 0;
        turretshooterconfig.MotionMagic.MotionMagicAcceleration = 1000;
        turretshooterconfig.Slot0.GravityType = GravityTypeValue.Elevator_Static;

        turretshooterconfig.MotorOutput.Inverted = InvertedValue.CounterClockwise_Positive;
        turretshooterLeft.getConfigurator().apply(turretshooterconfig);

        turretshooterconfig.MotorOutput.Inverted = InvertedValue.Clockwise_Positive;
        turretshooterRight.getConfigurator().apply(turretshooterconfig);

        TalonFXConfiguration turrethoodconfig = new TalonFXConfiguration();

        turrethoodconfig.CurrentLimits.StatorCurrentLimit = 10;
        turrethoodconfig.MotorOutput.NeutralMode = NeutralModeValue.Brake;
        turrethoodconfig.MotorOutput.Inverted = InvertedValue.Clockwise_Positive;
        turrethoodconfig.Slot0.kG = 0.0;
        turrethoodconfig.Slot0.kS = 0.0;
        turrethoodconfig.Slot0.kV = 0.0;
        turrethoodconfig.Slot0.kA = 0.0;
        turrethoodconfig.Slot0.kP = 3.0;
        turrethoodconfig.Slot0.kI = 0.0;
        turrethoodconfig.Slot0.kD = 0.0;

        turrethood.getConfigurator().apply(turrethoodconfig);

        turrethood.getPosition().setUpdateFrequency(100);

        // // Field
        // //
        // Positions-----------------------------------------------------------------------------------------------------------------------------------

        // robotAngleFromTag9 =
        // hubCenterLocation.minus(swerve.getPose().getTranslation()).getAngle().getDegrees();

        // robotDistanceToTag9 = Math.sqrt(Math.pow(swerve.getPose().getX() -
        // hubCenterLocation.getX(), 2)
        // + Math.pow(swerve.getPose().getY() - hubCenterLocation.getY(), 2));

        // // Calculates the position of the turret on the field by taking the turret's
        // // distance and angle (relative to the where the robot's facing) from the
        // center
        // // point of the robot
        // turretPositionX = swerve.getPose().getX()
        // + Math.cos(swerve.getPose().getTranslation().getAngle().getDegrees() +
        // turretDegreeFromRobotCenter)
        // * turretDistanceToRobotCenter;
        // turretPositionY = swerve.getPose().getY()
        // + Math.sin(swerve.getPose().getTranslation().getAngle().getDegrees() +
        // turretDegreeFromRobotCenter)
        // * turretDistanceToRobotCenter;

        // turretAngleFromTag9 = Math.atan2(hubCenterLocation.getX() - turretPositionX,
        // hubCenterLocation.getY() - turretPositionY);

        // turretDistanceToTag9 = Math.sqrt(Math.pow(turretPositionX -
        // hubCenterLocation.getX(), 2)
        // + Math.pow(turretPositionY - hubCenterLocation.getY(), 2));

        // /*
        // * When we shoot fuel while the robot is moving, the fuel's speed moves
        // relative
        // * to the robot's speed, throwing us off.
        // * To counteract this, we can calculate the time it will take for the fuel to
        // * reach the goal, and project the robot's position at that point. (current
        // * position + ( current velocity * the fuel goal time ))
        // * If we point the turret in the direction of the hub based from that position
        // * instead of it's current position, it should line up.
        // */

        // // Projects the time it will take for the feul to reach the goal when the
        // robot
        // // shoots
        // projectedTime = robotDistanceToTag9 * Math.cos(getTurretHood());

        // // Projects the turret's projected location relative to the field
        // projectedTurretPositionX = swerve.getPose().getX()
        // + (swerve.getRobotVelocity().vxMetersPerSecond * projectedTime)
        // + Math.cos(swerve.getPose().getTranslation().getAngle().getDegrees() +
        // turretDegreeFromRobotCenter)
        // * turretDistanceToRobotCenter;
        // projectedTurretPositionY = swerve.getPose().getY()
        // + (swerve.getRobotVelocity().vyMetersPerSecond * projectedTime)
        // + Math.cos(swerve.getPose().getTranslation().getAngle().getDegrees() +
        // turretDegreeFromRobotCenter)
        // * turretDistanceToRobotCenter;

        // // Projects the turret's angle from the hub
        // projectedTurretAngleFromTag9 = Math.atan2(hubCenterLocation.getX() -
        // projectedTurretPositionX,
        // hubCenterLocation.getY() - projectedTurretPositionY);
        // // Projects the turret's distance to the hub
        // projectedTurretDistanceToTag9 = Math.sqrt(Math.pow(projectedTurretPositionX -
        // hubCenterLocation.getX(), 2)
        // + Math.pow(projectedTurretPositionY - hubCenterLocation.getY(), 2));

        // getWrappedAngleDifference(swerve.getPose().getRotation().getDegrees(),
        // projectedTurretAngleFromTag9);

        // // set turret hood target position (Rudimentary calculation)
        // if (projectedTurretDistanceToTag9 < 1) {
        // } else {
        // }
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

    // sets speed of the shooter
    public void setShooterSpeedRPM(double rpm) {
        if (!turretEnabled) {
            return;
        }

        turretshooterLeft.setControl(new MotionMagicVelocityVoltage(RPM.of(rpm)));
        turretshooterRight.setControl(new MotionMagicVelocityVoltage(RPM.of(rpm)));
    }

    public double getShooter() {
        return turretshooterLeft.getPosition().getValueAsDouble() / 1.5;
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
    public void setTurretHood(double power) {
        if (!turretEnabled) {
            return;
        }

        if ((power > 0 && getTurretHood() > 80) || (power < 0 && getTurretHood() < 42)) {
            turrethood.set(0);
            return;
        }

        turrethood.set(power);
        return;

    }

    public void setTurretHoodUnchecked(double power) {
        if (!turretEnabled) {
            return;
        }

        turrethood.set(power);
    }

    public double getTurretHoodCurrent() {
        return turrethood.getStatorCurrent().getValue().in(Amps);
    }

    public void resetHoodPosition() {
        turrethood.setPosition(0);
    }

    // returns the current position of the hood
    public double getTurretHood() {
        return (turrethood.getPosition().getValueAsDouble() / 29.5 % 360);
    }

    // Estimates the angle we want to shoot the fuel at based on the turret's
    // distance to the hub (20 = angle the ball shoots at when the hood is at 80
    // degrees, not sure what is actually is)
    // The numbers used here arised from tinkering around to get an accurate
    // estimate equasion (28500 / ("Distance To Hub" + 25) ^ 2) + 46.25
    public double targetHoodAngle(double distance) {
        return 80 - 20 - ((28500 / (Math.pow(distance + 25, 2))) + 46.25);
    }

    // Estimates the speed we want to shoot the fuel at based on the turret's
    // distance to the hub
    public double targetShooterSpeed(double distance) {
        return (((Math.pow(distance + 12, 2)) * 0.0094) + 20.3)
                + (1 / (distance - 2.15));
    }

    public void setHoodToAngle(double angle) {

        if (Math.abs(getWrappedAngleDifference(getTurretHood(), angle)) < 0.3) {
            setTurretHood(0);
            return;
        }

        setTurretHood(1 * getWrappedAngleDifference(getTurretHood(), angle));

    }

    @Override
    public void periodic() {
        SmartDashboard.putNumber("Left Shooter RPM", turretshooterLeft.getVelocity().getValue().in(RPM));
        SmartDashboard.putNumber("Right Shooter RPM", turretshooterRight.getVelocity().getValue().in(RPM));

        if (!turretEnabled) {
            turrethood.set(0);
            turretshooterLeft.set(0);
            turretshooterRight.set(0);
        }
    }

    public void setEnabled(boolean turretEnabled) {
        this.turretEnabled = turretEnabled;
    }

}