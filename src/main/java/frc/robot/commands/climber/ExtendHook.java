package frc.robot.commands.climber;

import frc.robot.Constants;
import frc.robot.subsystems.ClimberSubsystem;
import edu.wpi.first.wpilibj2.command.Command;

public class ExtendHook extends Command {
    private ClimberSubsystem climber;

    @Override
    public void initialize() {
        // Sets the Hook's desired level to 1
        climber.setDesiredLevel(1);
    }

    @Override
    public void execute() {
        // Set it for a positive PID, to extend upwards and slow towards the bar
        climber.setClimbMotor((Constants.ClimberConstants.BAR1_ELEVATION - climber.getHookElevation()) * Constants.ClimberConstants.EXTEND_P);
    }

    @Override
    public boolean isFinished() {
        // See if we're within a tolerance range
        return (Math.abs(climber.getHookElevation() - Constants.ClimberConstants.BAR1_ELEVATION) <= Constants.ClimberConstants.CLIMB_TOLERANCE);
    }

    @Override
    public void end(boolean interuppted) {
        // Turn off the motor
        climber.setClimbMotor(0);
    }
}
