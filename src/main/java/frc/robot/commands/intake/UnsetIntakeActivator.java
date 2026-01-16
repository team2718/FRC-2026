package frc.robot.commands.intake;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.intake.IntakeSubsystem;

public class UnsetIntakeActivator extends Command {
    private final IntakeSubsystem intake;
    // private final LEDSubsystem LEDs;
    private double speed;

    public UnsetIntakeActivator(IntakeSubsystem intake,  double speed) {
        this.intake = intake;
      
        this.speed = speed;
    }

    @Override
    public void initialize() {}

    @Override
    public void execute() {

        //If the intake setup is not set to the starting position, it will rotate until it reaches the position
        //If the setup goes to low to the ground, it rotates back up

        if (intake.getActivatorPosition() > 3) {
            intake.SetIntakeSpeed(-speed);
        } else if (intake.getActivatorPosition() < -3) {
            intake.SetIntakeSpeed(speed);
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
        return intake.atStartPosition();
    }
}
