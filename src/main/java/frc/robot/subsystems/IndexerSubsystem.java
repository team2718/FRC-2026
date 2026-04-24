package frc.robot.subsystems;

import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.VelocityVoltage;
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
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

@Logged
public class IndexerSubsystem extends SubsystemBase {

    @Logged(name = "Indexer Motor")
    private final SparkMax indexerMotor;
    @Logged(name = "Portal Motor")
    private final TalonFX portalMotor;

    private final VelocityVoltage portalVelocityRequest = new VelocityVoltage(0);

    private boolean enabled = true;

    public IndexerSubsystem() {
        indexerMotor = new SparkMax(Constants.IndexerConstants.INDEXER_MOTOR_ID, SparkLowLevel.MotorType.kBrushless);
        portalMotor = new TalonFX(Constants.IndexerConstants.PORTAL_MOTOR_ID);

        SparkMaxConfig indexerMotorConfig = new SparkMaxConfig();
        indexerMotorConfig.inverted(true);
        indexerMotorConfig.smartCurrentLimit(60);
        indexerMotorConfig.idleMode(IdleMode.kCoast);
        indexerMotorConfig.voltageCompensation(12);
        indexerMotorConfig.openLoopRampRate(0.1);

        TalonFXConfiguration portalMotorConfig = new TalonFXConfiguration();
        portalMotorConfig.MotorOutput.Inverted = InvertedValue.Clockwise_Positive;
        portalMotorConfig.CurrentLimits.StatorCurrentLimit = 40;
        portalMotorConfig.MotorOutput.NeutralMode = NeutralModeValue.Coast;

        portalMotorConfig.Slot0.kS = 0.1;
        portalMotorConfig.Slot0.kV = 0.115;
        portalMotorConfig.Slot0.kP = 0.10;
        portalMotorConfig.Slot0.kI = 0.0;
        portalMotorConfig.Slot0.kD = 0.0;

        indexerMotor.configure(indexerMotorConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
        portalMotor.getConfigurator().apply(portalMotorConfig);
    }

    public void runIndexing(AngularVelocity portalAngularVelocity) {
        if (enabled) {
            indexerMotor.setVoltage(6.0);
            portalMotor.setControl(portalVelocityRequest.withVelocity(portalAngularVelocity));
        }
    }

    public void stopIndexing() {
        indexerMotor.stopMotor();
        portalMotor.stopMotor();
    }

    public void setIndexerSpeed(double speed) {
        if (enabled) {
            indexerMotor.set(speed);
        }
    }

    public void setIndexerVoltage(double voltage) {
        if (enabled) {
            indexerMotor.setVoltage(voltage);
        }
    }

    public void setPortalVoltage(double voltage) {
        if (enabled) {
            portalMotor.setVoltage(voltage);
        }
    }

    public void setPortalSpeed(double speed) {
        if (enabled) {
            portalMotor.set(speed);
        }
    }

    @Override
    public void periodic() {
        if (!enabled) {
            indexerMotor.stopMotor();
            portalMotor.stopMotor();
            return;
        }
    }

    public void setEnabled(boolean indexerIntakeEnabled) {
        this.enabled = indexerIntakeEnabled;
    }
}