package frc.robot.commands.intake;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.IndexerSubsystem;
import frc.robot.subsystems.IntakeSubsystem;

public class RunIntake extends Command {
    private final IntakeSubsystem intake;
    private final IndexerSubsystem indexer;
    private double speed;

    private Timer timer = new Timer();

    public RunIntake(IntakeSubsystem intake, IndexerSubsystem indexer, double speed) {
        this.intake = intake;
        this.indexer = indexer;
        this.speed = speed;
        addRequirements(intake, indexer);
    }

    @Override
    public void initialize() {
        timer.reset();
        timer.start();
    }

    @Override
    public void execute() {
        intake.setIntakeSpeed(speed);

        // Run indexer on sine wave to prevent jamming
        // Go between 0.2 and -0.2 speed every 0.5 seconds
        double indexerSpeed = 0.2 * Math.sin(2 * Math.PI * timer.get() / 0.5);
        indexer.setIndexerSpeed(indexerSpeed);
    }

    @Override
    public void end(boolean interuppted) {
        intake.setIntakeSpeed(0);
        indexer.setIndexerSpeed(0);
    }

    @Override
    public boolean isFinished() {
        return false;
    }
}
