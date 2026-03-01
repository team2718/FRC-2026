package frc.robot.commands.climber;

import frc.robot.subsystems.ClimberSubsystem;
import edu.wpi.first.wpilibj2.command.Command;

public class ZeroClimber extends Command {
    private ClimberSubsystem climber;

    public ZeroClimber(ClimberSubsystem climber) {
        this.climber = climber;
        addRequirements(climber);
    }

    @Override
    public void initialize() {
        // Set desired level to 0, and reduce current limit
        // climber.setDesiredLevel(0);
        climber.applyZeroingMotorConfig();
    }

    @Override
    public void execute() {
        // Set a small negative voltage
        climber.setClimbMotorVoltage(-3);
    }

    @Override
    public boolean isFinished() {
        // Check if the current is near the stall limit
        return (climber.getAmps() >= 18);
    }

    @Override
    public void end(boolean interuppted) {
        // Stop the motor, reset the current limit, and set the default hook elevation
        climber.applyStandardMotorConfig();
        climber.setClimbMotor(0);
        climber.resetHookElevation();
    }
}
