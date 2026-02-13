package frc.robot.commands.indexer;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.IndexerSubsystem;

public class SpinIndexerBackward extends Command {
    private final IndexerSubsystem indexer;
    // private final LEDSubsystem LEDs;
    private double speed;

    public SpinIndexerBackward(IndexerSubsystem indexer,  double speed) {
        this.indexer = indexer;
      
        this.speed = speed;
    }

    @Override
    public void initialize() {}

    @Override
    public void execute() {

        //Set indexer Speed
        indexer.setSpeed(-speed);
        
    }

    @Override
    public void end(boolean interuppted) {
        indexer.setSpeed(0);
    }

    @Override
    public boolean isFinished() {
        return false;
    }
}