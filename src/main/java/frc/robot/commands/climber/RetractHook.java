package frc.robot.commands.climber;

import frc.robot.Constants;
import frc.robot.subsystems.ClimberSubsystem;
import edu.wpi.first.wpilibj2.command.Command;

public class RetractHook extends Command {
    private ClimberSubsystem climber;

    @Override
    public void initialize() {
        climber.setDesiredLevel(0);
    }

    @Override
    public void execute() {
        climber.setClimbMotor(climber.getHookElevation() * -1 * Constants.ClimberConstants.CLIMBER_P);
    }

    @Override
    public boolean isFinished() {
        return (climber.getHookElevation() <= 0.1);
    }

    @Override
    public void end(boolean interuppted) {
        climber.setClimbMotor(0);
    }
}
