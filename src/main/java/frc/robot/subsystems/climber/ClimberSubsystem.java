package frc.robot.subsystems.climber;

import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.SparkMax;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.Constants;

public class ClimberSubsystem {
    // Fields
    private final SparkMax climbMotor;
    private boolean isClimbing = false;
    private boolean isReleasing = false;
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

    // Constructors
    public ClimberSubsystem() {
        climbMotor = new SparkMax(0, MotorType.kBrushless);
    }

    // Methods
    public void periodic() {
        SmartDashboard.putNumber("ClimbMotor Encoder", getClimbMotorPosition());
        SmartDashboard.putBoolean("Climbing?", isClimbing);
        SmartDashboard.putBoolean("Releasing?", isReleasing);
    }

    public void climb(int level) {
        if (!isReleasing) {
            if (isClimbing) {
                // L1 Climb
                if ((desiredLevel == 1 && robotElevation < Constants.ClimberConstants.L1_ELEVATION) || 
                (desiredLevel > 1 && robotElevation < Constants.ClimberConstants.BAR1_ELEVATION)) {
                    if (hookElevation < Constants.ClimberConstants.BAR1_ELEVATION) {
                        climbMotor.set((Constants.ClimberConstants.BAR1_ELEVATION - hookElevation) * Constants.ClimberConstants.CLIMBER_P);
                    }
                    else {
                        climbMotor.set((Constants.ClimberConstants.BAR1_ELEVATION - hookElevation) * -1 * Constants.ClimberConstants.CLIMBER_P);
                    }
                }
                // L2 Climb
                else if ((desiredLevel == 2 && robotElevation < Constants.ClimberConstants.L2_ELEVATION) ||
                (desiredLevel > 2 && robotElevation < Constants.ClimberConstants.BAR2_ELEVATION)) {
                    if (hookElevation < Constants.ClimberConstants.BAR2_ELEVATION) {
                        climbMotor.set((Constants.ClimberConstants.BAR2_ELEVATION - hookElevation) * Constants.ClimberConstants.CLIMBER_P);
                    }
                    else {
                        climbMotor.set((Constants.ClimberConstants.BAR2_ELEVATION - hookElevation) * -1 * Constants.ClimberConstants.CLIMBER_P);
                    }
                }
                // L3 Climb
                else if (desiredLevel == 3 && robotElevation < Constants.ClimberConstants.L3_ELEVATION) {
                    if (hookElevation < Constants.ClimberConstants.BAR3_ELEVATION) {
                        climbMotor.set((Constants.ClimberConstants.BAR3_ELEVATION - hookElevation) * Constants.ClimberConstants.CLIMBER_P);
                    }
                    else {
                        climbMotor.set((Constants.ClimberConstants.BAR3_ELEVATION - hookElevation) * -1 * Constants.ClimberConstants.CLIMBER_P);
                    }    
                }
            }
            else {
                // Set the robot's desired elevation
                isClimbing = true;
                desiredLevel = level;
            }
        }
    }

    public void release() {
        if (!isClimbing && !isReleasing) {
            isReleasing = true;
            desiredLevel = 0;
        }
    }
}
