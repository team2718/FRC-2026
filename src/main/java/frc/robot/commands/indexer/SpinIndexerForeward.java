package frc.robot.commands.indexer;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.IndexerSubsystem;

public class SpinIndexerForeward extends Command {
    private final IndexerSubsystem indexer;
    // private final LEDSubsystem LEDs;
    private double voltage;

    public SpinIndexerForeward(IndexerSubsystem indexer, double voltage) {
        this.indexer = indexer;
        this.voltage = voltage;
        addRequirements(indexer);
    }

    @Override
    public void initialize() {
    }

    @Override
    public void execute() {

        // Set indexer Speed
        indexer.setIndexerVoltage(voltage);
        indexer.setPortalVoltage(voltage);
    }

    @Override
    public void end(boolean interuppted) {
        indexer.setIndexerVoltage(0);
        indexer.setPortalVoltage(0);
    }

    @Override
    public boolean isFinished() {
        return false;
    }
}