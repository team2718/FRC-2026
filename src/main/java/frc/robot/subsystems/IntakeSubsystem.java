package frc.robot.subsystems;

import com.revrobotics.spark.SparkLowLevel;
import com.revrobotics.spark.SparkMax;
//import com.revrobotics.spark.config.SparkParameters;

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
    private final SparkMax intakeactivator;

    // shuffleboard
    private ShuffleboardTab comptab = Shuffleboard.getTab("intake");
    //sensors 
    private GenericEntry intakeSwitch = comptab.add("intake switch", false).getEntry();

public IntakeSubsystem() {
    intakemotor = new SparkMax(Constants.IntakeConstants.intakemotorID, SparkLowLevel.MotorType.kBrushless);
    intakeactivator = new SparkMax(Constants.IntakeConstants.intakeactivatorID, SparkLowLevel.MotorType.kBrushless);
}


//sets activator speed foreward
public void setActivatorSpeed(double power) {
    intakeactivator.set(power);
}

//stops activator
public void stopActivator(double power) {
    intakeactivator.set(0);
}

//sets intake speed 
public void setSpeed(double power) {
    intakemotor.set(power);
}

//stops intake
public void stopIntake() {
    intakemotor.set(0);
}

//for backwards intake
public void setbackSpeed(double power) {
    intakemotor.set(-power);
}

//returns the current speed of the intake motor
public double getIntakeSpeed() {
    return intakemotor.get();
}

//returns the current speed of the activator motor
public double getActivatorSpeed() {
    return intakeactivator.get();
}

//returns the current position of the activator motor
public double getActivatorPosition() {
    return intakeactivator.getAbsoluteEncoder().getPosition();
}

public boolean atStartPosition() {
    return intakeactivator.getAbsoluteEncoder().getPosition() > -3 && intakeactivator.getAbsoluteEncoder().getPosition() < 3;
}

public boolean atEndPosition() {
    return intakeactivator.getAbsoluteEncoder().getPosition() > 117 && intakeactivator.getAbsoluteEncoder().getPosition() < 123;
}

@Override
public void periodic() {
     intakeSwitch.setBoolean(true);
}

//sets intake in commmand
public void SetIntakeSpeed(double speed) {
    intakemotor.set(speed);
} 

}