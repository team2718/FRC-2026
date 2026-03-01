package frc.robot.subsystems;

import com.revrobotics.PersistMode;
import com.revrobotics.ResetMode;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;
import com.revrobotics.spark.config.SparkMaxConfig;

import edu.wpi.first.epilogue.Logged;
import edu.wpi.first.wpilibj.Alert;
import edu.wpi.first.wpilibj.Alert.AlertType;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

@Logged
public class ClimberSubsystem extends SubsystemBase {
    // Fields
    @Logged(name = "Climb Motor")
    private final SparkMax climbMotor;
    private SparkMaxConfig climbMotorConfig;
    private Alert climbMotorAlert;
    private boolean climbing = false;
    private boolean releasing = false;
    private int desiredLevel = 0;
    private double robotElevation = 0;
    private double hookElevation = 0;

    private boolean enabled = true;

    public void applyStandardMotorConfig() {
        climbMotorConfig.smartCurrentLimit(Constants.ClimberConstants.CURRENT_LIMIT);
        climbMotorConfig.openLoopRampRate(0.25);
        climbMotorConfig.softLimit.forwardSoftLimitEnabled(true);
        climbMotorConfig.softLimit.reverseSoftLimitEnabled(true);
        climbMotor.configure(climbMotorConfig, ResetMode.kNoResetSafeParameters, PersistMode.kNoPersistParameters);
    }

    public void applyZeroingMotorConfig() {
        climbMotorConfig.smartCurrentLimit(20);
        climbMotorConfig.openLoopRampRate(0.8);
        climbMotorConfig.softLimit.forwardSoftLimitEnabled(false);
        climbMotorConfig.softLimit.reverseSoftLimitEnabled(false);
        climbMotor.configure(climbMotorConfig, ResetMode.kNoResetSafeParameters, PersistMode.kNoPersistParameters);
    }

    public double getAmps() {
        return climbMotor.getOutputCurrent();
    }

    public double getClimbMotorPosition() {
        return climbMotor.getEncoder().getPosition();
    }

    public void setClimbMotorVoltage(double voltage) {
        if (enabled) {
            climbMotor.setVoltage(voltage);
        }
    }

    public void setClimbMotor(double speed) {
        if (enabled) {
            climbMotor.set(speed);
        }
    }

    public double getRobotElevation() {
        return robotElevation;
    }

    public double getHookElevation() {
        return hookElevation;
    }

    public void resetHookElevation() {
        climbMotor.getEncoder().setPosition(0);
        System.out.println("Hook position reset!");
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
        climbMotor = new SparkMax(Constants.ClimberConstants.MOTOR_ID, MotorType.kBrushless);
        climbMotorConfig = new SparkMaxConfig();
        climbMotorConfig.idleMode(IdleMode.kBrake);
        climbMotorConfig.inverted(true); // Up should be positive
        climbMotorConfig.smartCurrentLimit(Constants.ClimberConstants.CURRENT_LIMIT);
        climbMotorConfig.openLoopRampRate(0.25);

        climbMotorConfig.softLimit.forwardSoftLimit(Constants.ClimberConstants.FORWARD_SOFT_LIMIT);
        climbMotorConfig.softLimit.reverseSoftLimit(0.0);
        climbMotorConfig.softLimit.forwardSoftLimitEnabled(true);
        climbMotorConfig.softLimit.reverseSoftLimitEnabled(true);

        climbMotor.configure(climbMotorConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
        climbMotorAlert = new Alert("Motor \"Climb Motor\" is faulting!", AlertType.kError);
    }

    // Methods
    public void updateElevations() {
        // Need to do some math here to translate encoder rotation to height raised
        hookElevation = Constants.ClimberConstants.HOOK_BASE_ELEVATION + getClimbMotorPosition();
    }

    public void periodic() {
        SmartDashboard.putNumber("Climber Current", climbMotor.getOutputCurrent());
        SmartDashboard.putNumber("Climber Output", climbMotor.getAppliedOutput());
        SmartDashboard.putNumber("Climber Input Voltage", climbMotor.getBusVoltage());

        if (!enabled) {
            climbMotor.set(0);
        }
    }

    public void setAlerts() {
        climbMotorAlert.set(climbMotor.hasActiveFault());
    }

    public void setEnabled(boolean climberEnabled) {
        enabled = climberEnabled;
    }
}