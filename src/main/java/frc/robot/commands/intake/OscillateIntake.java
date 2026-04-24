package frc.robot.commands.intake;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.IntakeArmSubsystem;

public class OscillateIntake extends Command {
    private final IntakeArmSubsystem intakeArm;
    private final Timer timer = new Timer();

    private final static double OSCILLATION_FREQUENCY = 1.0;
    private final static double OSCILLATION_AMPLITUDE = 20.0;

    public OscillateIntake(IntakeArmSubsystem intakeArm) {
        this.intakeArm = intakeArm;
        addRequirements(intakeArm);
    }

    @Override
    public void initialize() {
        timer.reset();
        timer.start();
    }

    @Override
    public void execute() {
        // Follow a sine wave pattern to oscillate the intake's position
        double angle = OSCILLATION_AMPLITUDE * (1 + Math.sin(2 * Math.PI * OSCILLATION_FREQUENCY * timer.get()));
        intakeArm.setSlapdownPosition(angle);
    }

    @Override
    public void end(boolean interrupted) {
        intakeArm.useAutomaticPositioning(); // Return to automatic control when the command ends
    }

     @Override
     public boolean isFinished() {
         return false; // This command runs until interrupted
     }
}