package frc.robot.commands.intake;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.IntakeSubsystem;

public class RunIntake extends Command {
    private final IntakeSubsystem intake;
    // private final LEDSubsystem LEDs;
    private double speed;

    public RunIntake(IntakeSubsystem intake,  double speed) {
        this.intake = intake;
      
        this.speed = speed;
    }

    @Override
    public void initialize() {}

    @Override
    public void execute() {

        //Sets the speed of the intake foreward
        intake.setIntakeVoltage(speed);

        
    }

    @Override
    public void end(boolean interuppted) {
        intake.setIntakeVoltage(0);
    }

    @Override
    public boolean isFinished() {
        return true;
    }
}
