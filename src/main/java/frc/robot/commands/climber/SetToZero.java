package frc.robot.commands.climber;

import frc.robot.Constants;
import frc.robot.subsystems.ClimberSubsystem;
import edu.wpi.first.wpilibj2.command.Command;

public class SetToZero extends Command {
    private ClimberSubsystem climber;

    public SetToZero(ClimberSubsystem climber) {
        this.climber = climber;
    }

    @Override
    public void initialize() {
        // Set desired level to 0
        climber.setDesiredLevel(0);
    }

    @Override
    public void execute() {
        // Set PID to inverse; use a stronger P than for extension, to account for the robot weight
        climber.setClimbMotorVoltage(-5);
    }

    @Override
    public boolean isFinished() {
        return false;
    }

    @Override
    public void end(boolean interuppted) {
        // Stop the motor
        climber.setClimbMotor(0);
        climber.resetHookElevation();
    }
}
