package frc.robot.utils;

import static edu.wpi.first.units.Units.Amps;
import static edu.wpi.first.units.Units.Celsius;
import static edu.wpi.first.units.Units.RPM;
import static edu.wpi.first.units.Units.Volts;

import com.ctre.phoenix6.hardware.TalonFX;

import edu.wpi.first.epilogue.CustomLoggerFor;
import edu.wpi.first.epilogue.Epilogue;
import edu.wpi.first.epilogue.Logged;
import edu.wpi.first.epilogue.logging.ClassSpecificLogger;
import edu.wpi.first.epilogue.logging.EpilogueBackend;

@CustomLoggerFor(TalonFX.class)
public class TalonFXLogger extends ClassSpecificLogger<TalonFX> {
  public TalonFXLogger() {
    super(TalonFX.class);
  }

  @Override
  public void update(EpilogueBackend backend, TalonFX motor) {
    // Critical logs

    if (!Epilogue.shouldLog(Logged.Importance.INFO)) {
      return;
    }
    // Info logs

    backend.log("Output Duty Cycle", motor.getDutyCycle().getValue().doubleValue());
    backend.log("Supply Voltage (V)", motor.getSupplyVoltage().getValue().in(Volts));
    backend.log("Output Voltage (V)", motor.getMotorVoltage().getValue().in(Volts));
    backend.log("Stator Current (A)", motor.getStatorCurrent().getValue().in(Amps));
    backend.log("Supply Current (A)", motor.getSupplyCurrent().getValue().in(Amps));
    backend.log("Velocity (RPM)", motor.getVelocity().getValue().in(RPM));

    if (!Epilogue.shouldLog(Logged.Importance.DEBUG)) {
      return;
    }
    // Debug logs

    backend.log("Temperature (C)", motor.getDeviceTemp().getValue().in(Celsius));
  }
}