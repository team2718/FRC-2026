package frc.robot.subsystems;

import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;
import com.revrobotics.PersistMode;
import com.revrobotics.ResetMode;
import com.revrobotics.spark.SparkLowLevel;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;
import com.revrobotics.spark.config.SparkMaxConfig;

import edu.wpi.first.epilogue.Logged;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

@Logged
public class IntakeSubsystem extends SubsystemBase {

    private final TalonFX intakeMotor;
    private final SparkMax slapdownMotor;

    private final double stowedAngle = 0;
    private final double activeAngle = 90;

    private boolean setStowed = false;

    private boolean enabled = true;

    public IntakeSubsystem() {
        intakeMotor = new TalonFX(Constants.IntakeConstants.INTAKE_MOTOR_ID);
        slapdownMotor = new SparkMax(Constants.IntakeConstants.SLAPDOWN_MOTOR_ID, SparkLowLevel.MotorType.kBrushless);

        SparkMaxConfig slapdownMotorConfig = new SparkMaxConfig();
        slapdownMotorConfig.inverted(false);
        slapdownMotorConfig.smartCurrentLimit(15);
        slapdownMotorConfig.idleMode(IdleMode.kBrake);
        // slapdownMotorConfig.absoluteEncoder.zeroCentered(true);
        // slapdownMotorConfig.absoluteEncoder.zeroOffset(0.5); // TODO: find the
        // correct offset for the absolute encoder
        // slapdownMotorConfig.closedLoop.pid(0.1, 0.0, 0.0); // TODO: tune the PID
        // values for the slapdown motor
        // slapdownMotorConfig.closedLoop.maxMotion.cruiseVelocity(RotationsPerSecond.of(1).in(RPM));
        // // TODO: find the correct cruise velocity for the slapdown motor
        // slapdownMotorConfig.closedLoop.maxMotion.maxAcceleration(RotationsPerSecondPerSecond.of(0.1).in(RPM.per(Second)));
        // // TODO: find the correct acceleration for the slapdown motor
        slapdownMotor.configure(slapdownMotorConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);

        TalonFXConfiguration intakeMotorConfig = new TalonFXConfiguration();
        intakeMotorConfig.MotorOutput.Inverted = InvertedValue.CounterClockwise_Positive;
        intakeMotorConfig.CurrentLimits.StatorCurrentLimit = 20;
        intakeMotorConfig.MotorOutput.NeutralMode = NeutralModeValue.Coast;
        intakeMotor.getConfigurator().apply(intakeMotorConfig);
    }

    public void setIntakeSpeed(double speed) {
        if (enabled) {
            intakeMotor.set(speed);
        }
    }

    public void setIntakeVoltage(double voltage) {
        if (enabled) {
            intakeMotor.setVoltage(voltage);
        }
    }

    public void setStowed() {
        setStowed = true;
    }

    public void setActive() {
        setStowed = false;
    }

    // gets the angle of the positioner motor
    public double getSlapdownAngleDegrees() {
        return slapdownMotor.getAbsoluteEncoder().getPosition() * 360;
    }

    @Override
    public void periodic() {
        // SmartDashboard.putNumber("Intake/Slapdown Angle", getSlapdownAngleDegrees());
        // SmartDashboard.putNumber("Intake/Slapdown Setpoint",
        // slapdownMotor.getClosedLoopController().getMAXMotionSetpointPosition() *
        // 360);

        if (!enabled) {
            intakeMotor.stopMotor();
            slapdownMotor.stopMotor();
            return;
        }

        // if (setStowed) {
        // slapdownMotor.getClosedLoopController().setSetpoint(stowedAngle,
        // SparkMax.ControlType.kMAXMotionPositionControl);
        // } else {
        // slapdownMotor.getClosedLoopController().setSetpoint(activeAngle,
        // SparkMax.ControlType.kMAXMotionPositionControl);
        // }
    }

    public void setEnabled(boolean indexerIntakeEnabled) {
        this.enabled = indexerIntakeEnabled;
    }

}