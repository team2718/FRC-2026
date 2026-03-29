package frc.robot.subsystems;

import static edu.wpi.first.units.Units.RPM;
import static edu.wpi.first.units.Units.RotationsPerSecond;
import static edu.wpi.first.units.Units.RotationsPerSecondPerSecond;
import static edu.wpi.first.units.Units.Second;

import com.revrobotics.PersistMode;
import com.revrobotics.ResetMode;
import com.revrobotics.spark.FeedbackSensor;
import com.revrobotics.spark.SparkLowLevel;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;
import com.revrobotics.spark.config.SparkMaxConfig;

import edu.wpi.first.epilogue.Logged;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

@Logged
public class IntakeArmSubsystem extends SubsystemBase {

    @Logged(name = "Slapdown Motor")
    private final SparkMax slapdownMotor;

    private final double stowedAngle = 96;
    private final double activeAngle = 0;

    private boolean manualPositioning = false;
    private boolean setStowed = false;

    private boolean enabled = true;

    public IntakeArmSubsystem() {
        slapdownMotor = new SparkMax(Constants.IntakeConstants.SLAPDOWN_MOTOR_ID, SparkLowLevel.MotorType.kBrushless);

        SparkMaxConfig slapdownMotorConfig = new SparkMaxConfig();
        slapdownMotorConfig.inverted(true);
        slapdownMotorConfig.smartCurrentLimit(40);
        slapdownMotorConfig.idleMode(IdleMode.kCoast);
        slapdownMotorConfig.closedLoop.feedbackSensor(FeedbackSensor.kAbsoluteEncoder);
        slapdownMotorConfig.absoluteEncoder.zeroCentered(true);
        slapdownMotorConfig.absoluteEncoder.zeroOffset(0.412); // TODO: find the
        slapdownMotorConfig.closedLoop.pid(2.5, 0.0, 0.0).outputRange(-0.2, 0.9); // TODO: tune the PID
        slapdownMotorConfig.closedLoop.maxMotion.cruiseVelocity(RotationsPerSecond.of(1).in(RPM));
        slapdownMotorConfig.closedLoop.maxMotion.maxAcceleration(RotationsPerSecondPerSecond.of(5).in(RPM.per(Second)));
        slapdownMotor.configure(slapdownMotorConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
    }

    public void setStowed() {
        setStowed = true;
    }

    public void setActive() {
        setStowed = false;
    }

    public void setSlapdownPosition(double angleDegrees) {
        manualPositioning = true;
        slapdownMotor.getClosedLoopController().setSetpoint(angleDegrees / 360.0, SparkMax.ControlType.kPosition);
    }

    public void useAutomaticPositioning() {
        manualPositioning = false;
    }

    // gets the angle of the positioner motor
    public double getSlapdownAngleDegrees() {
        return slapdownMotor.getAbsoluteEncoder().getPosition() * 360;
    }

    @Override
    public void periodic() {

        if (!enabled) {
            slapdownMotor.stopMotor();
            return;
        }

        if (manualPositioning) {
            return; // If we're in manual positioning mode, don't override the setpoint
        }

        if (setStowed) {
            slapdownMotor.getClosedLoopController().setSetpoint(stowedAngle / 360.0,
                    SparkMax.ControlType.kPosition);
        } else {
            slapdownMotor.getClosedLoopController().setSetpoint(activeAngle / 360.0,
                    SparkMax.ControlType.kPosition);
        }
    }

    public void setEnabled(boolean indexerIntakeEnabled) {
        this.enabled = indexerIntakeEnabled;
    }
}