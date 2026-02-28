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
    if(intake.getIntakeSpeed() < 0.1) {
        intake.setSpeed(speed);
    } else {
        intake.stopIntake();
    }
}

@Override
public boolean isFinished() {
    return true;
}







}
