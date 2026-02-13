package frc.robot.subsystems;

import com.revrobotics.PersistMode;
import com.revrobotics.ResetMode;
import com.revrobotics.spark.SparkLowLevel;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;
import com.revrobotics.spark.config.SparkMaxConfig;
import com.revrobotics.spark.config.SparkParameters;

import frc.robot.Constants;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.networktables.GenericEntry;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class IndexerSubsystem extends SubsystemBase{

    private final SparkMax indexermotor1;
    private final SparkMax indexermotor2;

    // shuffleboard
    private ShuffleboardTab comptab = Shuffleboard.getTab("indexer");

public IndexerSubsystem() {
    indexermotor1 = new SparkMax(Constants.IndexerConstants.indexermotor1ID, SparkLowLevel.MotorType.kBrushless);

    //Set Current Limit to 5 amps for now
    SparkMaxConfig indexerconfig1 = new SparkMaxConfig();
    indexerconfig1.inverted(false);
    indexerconfig1.smartCurrentLimit(5);
    indexerconfig1.idleMode(IdleMode.kCoast);

    indexermotor1.configure(indexerconfig1, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);

    indexermotor2 = new SparkMax(Constants.IndexerConstants.indexermotor2ID, SparkLowLevel.MotorType.kBrushless);

    SparkMaxConfig indexerconfig2 = new SparkMaxConfig();
    indexerconfig2.inverted(false);
    indexerconfig2.smartCurrentLimit(5);
    indexerconfig2.idleMode(IdleMode.kCoast);

    indexermotor2.configure(indexerconfig2, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
}


//sets indexer motor speed foreward
public void setSpeed(double power) {
    indexermotor1.set(power);
    indexermotor2.set(power);
}

//stops indexer
public void stopIndexer() {
    indexermotor1.set(0);
    indexermotor2.set(0);
}

//sets indexer motor speed foreward
public void setReverse(double power) {
    indexermotor1.set(-power);
    indexermotor2.set(-power);
}

//returns the current speed of the intake motor
public double getIntakeSpeed() {
    return indexermotor1.get();
}

@Override
public void periodic() {

}

}