package frc.robot.commands.intake;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.IntakeSubsystem;

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

        //Sets the speed of the intake foreward
        intake.SetIntakeSpeed(speed);

        
    }

    @Override
    public void end(boolean interuppted) {
        intake.setSpeed(0);
    }

    @Override
    public boolean isFinished() {
        return true;
    }
}
