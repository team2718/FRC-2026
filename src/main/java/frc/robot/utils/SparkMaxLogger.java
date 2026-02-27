package frc.robot.utils;

import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.SparkBase.Faults;

import edu.wpi.first.epilogue.CustomLoggerFor;
import edu.wpi.first.epilogue.Epilogue;
import edu.wpi.first.epilogue.Logged;
import edu.wpi.first.epilogue.logging.ClassSpecificLogger;
import edu.wpi.first.epilogue.logging.EpilogueBackend;

@CustomLoggerFor(SparkMax.class)
public class SparkMaxLogger extends ClassSpecificLogger<SparkMax> {
  public SparkMaxLogger() {
    super(SparkMax.class);
  }

  // Convert faults to a user-readable string
  private static String faultsToString(Faults faults) {
    StringBuilder sb = new StringBuilder();
    if (faults.other) sb.append("Other ");
    if (faults.motorType) sb.append("MotorType ");
    if (faults.sensor) sb.append("Sensor ");
    if (faults.can) sb.append("CAN ");
    if (faults.temperature) sb.append("Temperature ");
    if (faults.gateDriver) sb.append("GateDriver ");
    if (faults.escEeprom) sb.append("ESCEEPROM ");
    if (faults.firmware) sb.append("Firmware ");
    if (sb.length() == 0) {
      return "None";
    }
    return sb.toString().trim();
  }

  @Override
  public void update(EpilogueBackend backend, SparkMax motor) {
    // Critical logs
    backend.log("Faults", faultsToString(motor.getFaults()));

    if (!Epilogue.shouldLog(Logged.Importance.INFO)) {
      return;
    }
    // Info logs

    backend.log("Output Duty Cycle", motor.getAppliedOutput());
    backend.log("Bus Voltage (V)", motor.getBusVoltage());
    backend.log("Output Current (A)", motor.getOutputCurrent());
    backend.log("Velocity (RPM)", motor.getEncoder().getVelocity());

    if (!Epilogue.shouldLog(Logged.Importance.DEBUG)) {
      return;
    }
    // Debug logs

    backend.log("Temperature (C)", motor.getMotorTemperature());
  }
}