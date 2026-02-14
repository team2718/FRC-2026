package frc.robot.subsystems;

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

    // shuffleboard
    private ShuffleboardTab comptab = Shuffleboard.getTab("intake");
    //sensors 
    private GenericEntry intakeSwitch = comptab.add("intake switch", false).getEntry();

public IntakeSubsystem() {
    intakemotor = new SparkMax(Constants.IntakeConstants.intakemotorID, SparkLowLevel.MotorType.kBrushless);

    SparkMaxConfig intakeconfig = new SparkMaxConfig();
    intakeconfig.inverted(true);
    intakeconfig.smartCurrentLimit(5);
    intakeconfig.idleMode(IdleMode.kCoast);

    intakemotor.configure(intakeconfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
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

@Override
public void periodic() {
     intakeSwitch.setBoolean(true);
}

//sets intake in commmand
public void SetIntakeSpeed(double speed) {
    intakemotor.set(speed);
} 

}