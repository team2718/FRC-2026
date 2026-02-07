package frc.robot.subsystems;

import com.revrobotics.PersistMode;
import com.revrobotics.ResetMode;
import com.revrobotics.spark.SparkAbsoluteEncoder;
import com.revrobotics.spark.SparkLowLevel;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;
import com.revrobotics.spark.config.SparkMaxConfig;
import com.revrobotics.spark.config.SparkParameters;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.GravityTypeValue;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.DutyCycleOut;
import com.ctre.phoenix6.controls.Follower;

import frc.robot.Constants;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.networktables.GenericEntry;
import edu.wpi.first.wpilibj.Alert;
import edu.wpi.first.wpilibj.Alert.AlertType;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class TurretSubsystem extends SubsystemBase{

    private final SparkMax turretshooter;
    private final SparkMax turretspinner;
    private final TalonFX turrethood;

    // shuffleboard
    private ShuffleboardTab comptab = Shuffleboard.getTab("intake");
    //sensors 
    private GenericEntry intakeSwitch = comptab.add("intake switch", false).getEntry();

public TurretSubsystem() {
    turretshooter = new SparkMax(Constants.TurretConstants.turretshooterID, SparkLowLevel.MotorType.kBrushless);
    turretspinner = new SparkMax(Constants.TurretConstants.turretspinnerID, SparkLowLevel.MotorType.kBrushless);
    turrethood = new TalonFX(Constants.TurretConstants.turrethoodID);

    //Configuring motor variables (The current limit is set to 5 amps for now)

    SparkMaxConfig turretshooterconfig = new SparkMaxConfig();
    turretshooterconfig.inverted(false);
    turretshooterconfig.smartCurrentLimit(5); //5 amps
    turretshooterconfig.idleMode(IdleMode.kCoast);

    turretshooter.configure(turretshooterconfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);

    SparkMaxConfig turretspinnerconfig = new SparkMaxConfig();
    turretspinnerconfig.inverted(false);
    turretspinnerconfig.smartCurrentLimit(5); //5 amps
    turretspinnerconfig.idleMode(IdleMode.kCoast);

    turretspinner.configure(turretspinnerconfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);


    TalonFXConfiguration turrethoodconfig = new TalonFXConfiguration();

    //TO BE ADJUSTED (These parameters were just copied from last year's elevator subsystem <_<)

        turrethoodconfig.CurrentLimits.StatorCurrentLimit = 5;
        turrethoodconfig.MotorOutput.NeutralMode = NeutralModeValue.Brake;
        turrethoodconfig.MotorOutput.Inverted = InvertedValue.CounterClockwise_Positive;
        turrethoodconfig.Slot0.kG = 0.455;
        turrethoodconfig.Slot0.kS = 0.245;
        turrethoodconfig.Slot0.kV = 0.131;
        turrethoodconfig.Slot0.kA = 0.0011;
        turrethoodconfig.Slot0.kP = 1.8;
        turrethoodconfig.Slot0.kI = 0.0;
        turrethoodconfig.Slot0.kD = 0.0;
        turrethoodconfig.Slot0.GravityType = GravityTypeValue.Elevator_Static;
        turrethoodconfig.MotionMagic.MotionMagicCruiseVelocity = 60;
        turrethoodconfig.MotionMagic.MotionMagicAcceleration = 70;

        turrethood.getConfigurator().apply(turrethoodconfig);

        turrethood.getPosition().setUpdateFrequency(100);

}

public static double getWrappedAngleDifference(double source, double target) {
    double diff = (target - source) % 360;

    if (diff >= 180) {
        diff -= 360;
    }
    else if (diff <= -180) {
        diff += 360;
    }

    return diff;
}


//sets speed of the shooter
public void setShooterSpeed(double power) {
    turretshooter.set(power);
}

//sets rotational speed of the turret
public void setTurretPosition(double power) {
    turretspinner.set(power);
}

//returns the current rotational position of the turret
public double getTurretPosition() {
    return turretspinner.getEncoder().getPosition();
}

//sets rotational speed of the hood
public void setTurretHood(double power) {

    if ( (power > 0 && getTurretHood() > 80) || (power < 0 && getTurretHood() < 42) ) {
        turrethood.set(0);
        return; 
    }

    turrethood.set(power);
    return; 

}

//returns the current position of the hood
public double getTurretHood() {
    return (turrethood.getPosition().getValueAsDouble() / 29.5 % 360);
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
     intakeSwitch.setBoolean(true);
}

}