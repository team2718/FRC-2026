package frc.robot.commands.climber;

import frc.robot.Constants;
import frc.robot.subsystems.ClimberSubsystem;
import edu.wpi.first.wpilibj2.command.Command;

public class RetractHook extends Command {
    private ClimberSubsystem climber;

    @Override
    public void initialize() {
        // Set desired level to 0
        climber.setDesiredLevel(0);
    }

    @Override
    public void execute() {
        // Set PID to inverse; use a stronger P than for extension, to account for the robot weight
        climber.setClimbMotor(climber.getHookElevation() * -1 * Constants.ClimberConstants.RETRACT_P);
    }

    @Override
    public boolean isFinished() {
        // See if the hook is fully, or almost fully, retracted
        return (Math.abs(climber.getHookElevation() - Constants.ClimberConstants.HOOK_BASE_ELEVATION) <= Constants.ClimberConstants.CLIMB_TOLERANCE);
    }

    @Override
    public void end(boolean interuppted) {
        // Stop the motor
        climber.setClimbMotor(0);
    }
}
