package frc.robot.subsystems;

import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;

import edu.wpi.first.epilogue.Logged;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

@Logged
public class IntakeSubsystem extends SubsystemBase {

    @Logged(name = "Intake Motor")
    private final TalonFX intakeMotor;


    private boolean enabled = true;

    public IntakeSubsystem() {
        intakeMotor = new TalonFX(Constants.IntakeConstants.INTAKE_MOTOR_ID);

        TalonFXConfiguration intakeMotorConfig = new TalonFXConfiguration();
        intakeMotorConfig.MotorOutput.Inverted = InvertedValue.Clockwise_Positive;
        intakeMotorConfig.CurrentLimits.StatorCurrentLimit = 40;
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

    @Override
    public void periodic() {
        if (!enabled) {
            intakeMotor.stopMotor();
            return;
        }
    }

    public void setEnabled(boolean indexerIntakeEnabled) {
        this.enabled = indexerIntakeEnabled;
    }

    public void stop() {
        intakeMotor.stopMotor();
    }

}