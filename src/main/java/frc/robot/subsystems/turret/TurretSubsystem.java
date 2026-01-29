package frc.robot.subsystems.turret;

    import com.revrobotics.spark.SparkAbsoluteEncoder;
import com.revrobotics.spark.SparkLowLevel;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.config.SparkParameters;

import frc.robot.Constants;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.networktables.GenericEntry;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class TurretSubsystem extends SubsystemBase{

    private final SparkMax turretshooter;
    private final SparkMax turretspinner;

    // shuffleboard
    private ShuffleboardTab comptab = Shuffleboard.getTab("intake");
    //sensors 
    private GenericEntry intakeSwitch = comptab.add("intake switch", false).getEntry();

public TurretSubsystem() {
    turretshooter = new SparkMax(Constants.TurretConstants.turretshooterID, SparkLowLevel.MotorType.kBrushless);
    turretspinner = new SparkMax(Constants.TurretConstants.turretspinnerID, SparkLowLevel.MotorType.kBrushless);
}


//sets speed of the shooter
public void setShooterSpeed(double power) {
    turretshooter.set(power);
}

//sets rotational speed of the turret
public void setTurretSpin(double power) {
    turretspinner.set(power);
}

//returns the current rotational position of the turret
public double getTurretPosition() {
    return turretspinner.getEncoder().getPosition();
}

@Override
public void periodic() {
     intakeSwitch.setBoolean(true);
}

}