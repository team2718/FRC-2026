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
public class IndexerSubsystem extends SubsystemBase {

    private final SparkMax indexerMotor;
    private final TalonFX portalMotor;

    private boolean enabled = true;

    public IndexerSubsystem() {
        indexerMotor = new SparkMax(Constants.IndexerConstants.INDEXER_MOTOR_ID, SparkLowLevel.MotorType.kBrushless);
        portalMotor = new TalonFX(Constants.IndexerConstants.PORTAL_MOTOR_ID);

        SparkMaxConfig indexerMotorConfig = new SparkMaxConfig();
        indexerMotorConfig.inverted(false);
        indexerMotorConfig.smartCurrentLimit(30);
        indexerMotorConfig.idleMode(IdleMode.kCoast);

        TalonFXConfiguration portalMotorConfig = new TalonFXConfiguration();
        portalMotorConfig.MotorOutput.Inverted = InvertedValue.CounterClockwise_Positive;
        portalMotorConfig.CurrentLimits.StatorCurrentLimit = 15;
        portalMotorConfig.MotorOutput.NeutralMode = NeutralModeValue.Coast;

        indexerMotor.configure(indexerMotorConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
        portalMotor.getConfigurator().apply(portalMotorConfig);
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