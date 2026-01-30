package frc.robot.commands.intake;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.intake.IntakeSubsystem;

public class RunOuttake extends Command {
    private final IntakeSubsystem intake;
    // private final LEDSubsystem LEDs;
    private double speed;

    public RunOuttake(IntakeSubsystem intake,  double speed) {
        this.intake = intake;
      
        this.speed = speed;
    }

    @Override
    public void initialize() {}

    @Override
    public void execute() {

        /*If the intake wheel is set to the ground, ready to intake (If the setup is at the right position) the intake wheel can run
        Otherwise, the intake wheel cannot activate*/

        if (intake.atEndPosition()) {
            intake.SetIntakeSpeed(-speed);
        } else {
            intake.SetIntakeSpeed(0);
        }

        
    }

@Override
public void end(boolean interuppted) {
    intake.setSpeed(0);
    intake.setActivatorSpeed(0);
}

    @Override
    public boolean isFinished() {
        return true;
    }
}
