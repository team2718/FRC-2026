package frc.robot.subsystems;

import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.GravityTypeValue;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;
import com.revrobotics.PersistMode;
import com.revrobotics.ResetMode;
import com.revrobotics.spark.SparkLowLevel;
import com.revrobotics.spark.SparkMax;
//import com.revrobotics.spark.config.SparkParameters;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;
import com.revrobotics.spark.config.SparkMaxConfig;

import frc.robot.Constants;
//import edu.wpi.first.wpilibj2.command.Command;
//import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.networktables.GenericEntry;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
//import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class IntakeSubsystem extends SubsystemBase{

    private final SparkMax intakemotor;
    private final TalonFX intakepositioner;

    private final double stowedAngle;
    private final double activeAngle;

    // shuffleboard
    private ShuffleboardTab comptab = Shuffleboard.getTab("intake");
    //sensors 
    private GenericEntry intakeSwitch = comptab.add("intake switch", false).getEntry();

public IntakeSubsystem() {
    intakemotor = new SparkMax(Constants.IntakeConstants.intakemotorID, SparkLowLevel.MotorType.kBrushless);
    intakepositioner = new TalonFX(Constants.IntakeConstants.intakepositionerID);

    SparkMaxConfig intakeconfig = new SparkMaxConfig();
    intakeconfig.inverted(true);
    intakeconfig.smartCurrentLimit(5);
    intakeconfig.idleMode(IdleMode.kCoast);

    intakemotor.configure(intakeconfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);

    TalonFXConfiguration intakepositionerconfig = new TalonFXConfiguration();
    intakepositionerconfig.CurrentLimits.StatorCurrentLimit = 25;
    intakepositionerconfig.MotorOutput.NeutralMode = NeutralModeValue.Coast;

    intakepositionerconfig.Slot0.kG = 0.0;
    intakepositionerconfig.Slot0.kS = 0.0;
    intakepositionerconfig.Slot0.kV = 0.12;
    intakepositionerconfig.Slot0.kA = 0.0;

    intakepositionerconfig.Slot0.kP = 0.0;
    intakepositionerconfig.Slot0.kI = 0.0;
    intakepositionerconfig.Slot0.kD = 0.0;
    intakepositionerconfig.MotionMagic.MotionMagicCruiseVelocity = 0;
    intakepositionerconfig.MotionMagic.MotionMagicAcceleration = 1000;
    intakepositionerconfig.Slot0.GravityType = GravityTypeValue.Elevator_Static;

    intakepositionerconfig.MotorOutput.Inverted = InvertedValue.CounterClockwise_Positive;
    intakepositioner.getConfigurator().apply(intakepositionerconfig);

    stowedAngle = 135;
    activeAngle = 45;
}

//sets intake speed 
public void setSpeed(double power) {
    intakemotor.set(power);
}

//stops intake
public void stopIntake() {
    intakemotor.set(0);
}

//returns the current speed of the intake motor
public double getIntakeSpeed() {
    return intakemotor.get();
}

//if the intake is closer to one position, the function sets it to the other
public void setToTargetPosition() {
    if (getPositionerAngle() < 90) {
        intakepositioner.set(activeAngle - getPositionerAngle());
    } else {
        intakepositioner.set(stowedAngle - getPositionerAngle());
    }
}

//gets the angle of the positioner motor
public double getPositionerAngle() {
    return intakepositioner.getPosition().getValueAsDouble();
}


@Override
public void periodic() {
     intakeSwitch.setBoolean(true);
}

}