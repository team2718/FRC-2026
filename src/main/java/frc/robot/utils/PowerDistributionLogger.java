package frc.robot.utils;

import edu.wpi.first.epilogue.CustomLoggerFor;
import edu.wpi.first.epilogue.Epilogue;
import edu.wpi.first.epilogue.Logged;
import edu.wpi.first.epilogue.logging.ClassSpecificLogger;
import edu.wpi.first.epilogue.logging.EpilogueBackend;
import edu.wpi.first.wpilibj.PowerDistribution;

@CustomLoggerFor(PowerDistribution.class)
public class PowerDistributionLogger extends ClassSpecificLogger<PowerDistribution> {
  public PowerDistributionLogger() {
    super(PowerDistribution.class);
  }

  @Override
  public void update(EpilogueBackend backend, PowerDistribution pdh) {
    // Critical logs

    if (!Epilogue.shouldLog(Logged.Importance.INFO)) {
      return;
    }
    // Info logs

    backend.log("Total Current (A)", pdh.getTotalCurrent());
    backend.log("Voltage (V)", pdh.getVoltage());

    if (!Epilogue.shouldLog(Logged.Importance.DEBUG)) {
      return;
    }
    // Debug logs


  }
}