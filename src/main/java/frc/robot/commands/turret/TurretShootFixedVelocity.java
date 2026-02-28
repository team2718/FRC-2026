package frc.robot.commands.turret;

import static edu.wpi.first.units.Units.Degrees;

import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.IndexerSubsystem;
import frc.robot.subsystems.TurretSubsystem;

public class TurretShootFixedVelocity extends Command {
    private final TurretSubsystem shooter;
    private final IndexerSubsystem indexer;
    private final AngularVelocity angularVelocity;

    public TurretShootFixedVelocity(TurretSubsystem shooter, IndexerSubsystem indexer, AngularVelocity angularVelocity) {
        this.shooter = shooter;
        this.indexer = indexer;
        this.angularVelocity = angularVelocity;

        addRequirements(shooter, indexer);
    }

    @Override
    public void initialize() {
        
    }

    @Override
    public void execute() {
        shooter.setHoodAngle(Degrees.of(10));
        shooter.setShooterSpeed(angularVelocity);

        if (shooter.shooterAtSpeed()) {
            indexer.runIndexing();
        } else {
            indexer.stopIndexing();
        }
    }

    @Override
    public void end(boolean interuppted) {
        shooter.stopShooter();
        indexer.stopIndexing();
    }

    @Override
    public boolean isFinished() {
        return false;
    }
}