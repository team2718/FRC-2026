package frc.robot.commands.intake;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.IntakeArmSubsystem;
import frc.robot.subsystems.IntakeSubsystem;

public class AutoRunIntake extends Command {
    private final IntakeSubsystem intake;
    private final IntakeArmSubsystem intakeArm;
    private double speed;

    public AutoRunIntake(IntakeSubsystem intake, IntakeArmSubsystem intakeArm, double speed) {
        this.intake = intake;
        this.intakeArm = intakeArm;
        this.speed = speed;
    }

    @Override
    public void initialize() {
        intakeArm.setActive();
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
