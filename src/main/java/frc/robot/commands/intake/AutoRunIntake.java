package frc.robot.commands.intake;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.IntakeSubsystem;

public class AutoRunIntake extends Command {
    private final IntakeSubsystem intake;
    private double speed;

    public AutoRunIntake(IntakeSubsystem intake, double speed) {
        this.intake = intake;
        this.speed = speed;
    }

    @Override
    public void initialize() {
        intake.setActive();
        intake.setIntakeSpeed(speed);
    }

    @Override
    public void end(boolean interrupted) {
        // intake.setIntakeSpeed(0);
    }

    @Override
    public boolean isFinished() {
        return true;
    }

}
