package frc.robot.subsystems;

import static edu.wpi.first.units.Units.RPM;
import static edu.wpi.first.units.Units.RotationsPerSecond;
import static edu.wpi.first.units.Units.RotationsPerSecondPerSecond;
import static edu.wpi.first.units.Units.Second;

import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;
import com.revrobotics.PersistMode;
import com.revrobotics.ResetMode;
import com.revrobotics.spark.FeedbackSensor;
import com.revrobotics.spark.SparkLowLevel;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;
import com.revrobotics.spark.config.SparkMaxConfig;

import edu.wpi.first.epilogue.Logged;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

@Logged
public class IntakeSubsystem extends SubsystemBase {

    @Logged(name = "Intake Motor")
    private final TalonFX intakeMotor;
    @Logged(name = "Slapdown Motor")
    private final SparkMax slapdownMotor;

    private final double stowedAngle = 96;
    private final double activeAngle = 0;

    private boolean setStowed = true;

    private boolean enabled = true;

    public IntakeSubsystem() {
        intakeMotor = new TalonFX(Constants.IntakeConstants.INTAKE_MOTOR_ID);
        slapdownMotor = new SparkMax(Constants.IntakeConstants.SLAPDOWN_MOTOR_ID, SparkLowLevel.MotorType.kBrushless);

        SparkMaxConfig slapdownMotorConfig = new SparkMaxConfig();
        slapdownMotorConfig.inverted(true);
        slapdownMotorConfig.smartCurrentLimit(30);
        slapdownMotorConfig.idleMode(IdleMode.kCoast);
        slapdownMotorConfig.closedLoop.feedbackSensor(FeedbackSensor.kAbsoluteEncoder);
        slapdownMotorConfig.absoluteEncoder.zeroCentered(true);
        slapdownMotorConfig.absoluteEncoder.zeroOffset(0.412); // TODO: find the
        slapdownMotorConfig.closedLoop.pid(2.0, 0.0, 0.0).outputRange(-0.1, 0.9); // TODO: tune the PID
        slapdownMotorConfig.closedLoop.maxMotion.cruiseVelocity(RotationsPerSecond.of(1).in(RPM));
        slapdownMotorConfig.closedLoop.maxMotion.maxAcceleration(RotationsPerSecondPerSecond.of(5).in(RPM.per(Second)));
        slapdownMotor.configure(slapdownMotorConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);

        TalonFXConfiguration intakeMotorConfig = new TalonFXConfiguration();
        intakeMotorConfig.MotorOutput.Inverted = InvertedValue.Clockwise_Positive;
        intakeMotorConfig.CurrentLimits.StatorCurrentLimit = 30;
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
        SmartDashboard.putNumber("Intake/Slapdown Angle", getSlapdownAngleDegrees());

        if (!enabled) {
            intakeMotor.stopMotor();
            slapdownMotor.stopMotor();
            return;
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