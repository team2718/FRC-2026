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

    private final SparkMax indexermotor;

    // shuffleboard
    private ShuffleboardTab comptab = Shuffleboard.getTab("indexer");

public IndexerSubsystem() {
    indexermotor = new SparkMax(Constants.IndexerConstants.indexermotorID, SparkLowLevel.MotorType.kBrushless);

    SparkMaxConfig indexerconfig = new SparkMaxConfig();
    indexerconfig.inverted(false);
    indexerconfig.smartCurrentLimit(20);
    indexerconfig.idleMode(IdleMode.kCoast);

    indexermotor.configure(indexerconfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
}


//sets indexer motor speed foreward
public void setSpeed(double power) {
    indexermotor.set(power);
}

//stops indexer
public void stopIndexer() {
    indexermotor.set(0);
}

//sets indexer motor speed foreward
public void setReverse(double power) {
    indexermotor.set(-power);
}

//returns the current speed of the intake motor
public double getIntakeSpeed() {
    return indexermotor.get();
}

@Override
public void periodic() {

}

}