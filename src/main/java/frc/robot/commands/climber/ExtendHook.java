package frc.robot.commands.climber;

import frc.robot.Constants;
import frc.robot.subsystems.ClimberSubsystem;
import edu.wpi.first.wpilibj2.command.Command;

public class ExtendHook extends Command {
    private ClimberSubsystem climber;

    @Override
    public void initialize() {
        climber.setDesiredLevel(1);
    }

    @Override
    public void execute() {
        climber.setClimbMotor((Constants.ClimberConstants.BAR1_ELEVATION - climber.getHookElevation()) * Constants.ClimberConstants.CLIMBER_P);
    }

    @Override
    public boolean isFinished() {
        return (climber.getHookElevation() >= Constants.ClimberConstants.BAR1_ELEVATION);
    }

    @Override
    public void end(boolean interuppted) {
        climber.setClimbMotor(0);
    }
}
