package frc.robot.subsystems.climber;

import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.SparkMax;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.Constants;

public class ClimberSubsystem {
    // Fields
    private final SparkMax aMotor;
    private boolean isClimbing = false;
    private boolean isReleasing = false;
    private int level = 0;

    // Getters and Setters
    public double getAMotorPosition() {
        return aMotor.getEncoder().getPosition();
    }
    public void setAMotorVoltage(double voltage) {
        aMotor.setVoltage(voltage);
    }

    // Constructors
    public ClimberSubsystem() {
        aMotor = new SparkMax(0, MotorType.kBrushless);
    }

    // Methods
    public void periodic() {
        SmartDashboard.putNumber("aMotor Angle", getAMotorPosition());
        SmartDashboard.putBoolean("Climbing?", isClimbing);
    }

    public void climb(int level) {
        if (!isClimbing && !isReleasing) {
            isClimbing = true;
            // Do some climbing stuff
            this.level = level;
        }
    }

    public void release() {
        if (!isClimbing && !isReleasing) {
            isReleasing = true;
            // Release the climber
            level = 0;
        }
    }
}
