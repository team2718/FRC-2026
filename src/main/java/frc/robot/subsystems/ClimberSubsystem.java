package frc.robot.subsystems;

import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;
import com.revrobotics.spark.config.SparkMaxConfig;
import edu.wpi.first.wpilibj.Alert;
import edu.wpi.first.wpilibj.Alert.AlertType;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

public class ClimberSubsystem extends ExampleSubsystem {
    // Fields
    private final SparkMax climbMotor;
    private SparkMaxConfig climbMotorConfig;
    private Alert climbMotorAlert;
    private boolean climbing = false;
    private boolean releasing = false;
    private int desiredLevel = 0;
    private double robotElevation = 0;
    private double hookElevation = 0;

    // Getters and Setters
    public double getClimbMotorPosition() {
        return climbMotor.getEncoder().getPosition();
    }
    public void setClimbMotorVoltage(double voltage) {
        climbMotor.setVoltage(voltage);
    }
    public void setClimbMotor(double speed){
        climbMotor.set(speed);
    }
    public double getRobotElevation() {
        return robotElevation;
    }
    public double getHookElevation() {
        return hookElevation;
    }
    public int getDesiredLevel() {
        return desiredLevel;
    }
    public void setDesiredLevel(int desiredLevel) {
        this.desiredLevel = desiredLevel;
    }
    public boolean isClimbing() {
        return climbing;
    }
    public void setClimbing(boolean climbing) {
        this.climbing = climbing;
    }
    public boolean isReleasing() {
        return releasing;
    }

    // Constructors
    public ClimberSubsystem() {
        climbMotor = new SparkMax(0, MotorType.kBrushless);
        climbMotorConfig = new SparkMaxConfig();
        climbMotorConfig.idleMode(IdleMode.kBrake);
        climbMotorConfig.inverted(false);
        climbMotorConfig.smartCurrentLimit(20);
        climbMotorAlert = new Alert("Motor \"Climb Motor\" is faulting!", AlertType.kError);
    }

    // Methods
    public void periodic() {
        setAlerts();
        SmartDashboard.putNumber("ClimbMotor Encoder", getClimbMotorPosition());
        SmartDashboard.putBoolean("Climbing?", climbing);
        SmartDashboard.putBoolean("Releasing?", releasing);
    }
    public void setAlerts(){
        climbMotorAlert.set(climbMotor.hasActiveFault());
    }
}
