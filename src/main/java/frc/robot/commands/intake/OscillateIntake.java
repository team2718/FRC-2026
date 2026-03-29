package frc.robot.commands.intake;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.IntakeArmSubsystem;

public class OscillateIntake extends Command {
    private final IntakeArmSubsystem intakeArm;
    private final Timer timer = new Timer();

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
        // between 0 and 45 degrees at a frequency of 0.5 Hz (one full oscillation every 2 seconds)
        double angle = 22.5 * (1 + Math.sin(2 * Math.PI * 0.5 * timer.get())); // Oscillates between 0 and 45 degrees
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