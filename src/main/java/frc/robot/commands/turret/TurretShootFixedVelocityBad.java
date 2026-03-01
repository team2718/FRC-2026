package frc.robot.commands.turret;

import static edu.wpi.first.units.Units.Feet;
import static edu.wpi.first.units.Units.Meters;
import static edu.wpi.first.units.Units.RPM;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.units.measure.Distance;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Strategy;
import frc.robot.subsystems.IndexerSubsystem;
import frc.robot.subsystems.SwerveSubsystem;
import frc.robot.subsystems.TurretSubsystem;

public class TurretShootFixedVelocityBad extends Command {
    private final TurretSubsystem shooter;
    private final IndexerSubsystem indexer;
    private final SwerveSubsystem swerve;

    public TurretShootFixedVelocityBad(TurretSubsystem shooter, IndexerSubsystem indexer, SwerveSubsystem swerve) {
        this.shooter = shooter;
        this.indexer = indexer;
        this.swerve = swerve;

        SmartDashboard.putNumber("Target RPM", 2000);

        addRequirements(shooter, indexer);
    }

    @Override
    public void initialize() {
        
    }

    @Override
    public void execute() {

        AngularVelocity angularVelocity = RPM.of(SmartDashboard.getNumber("Target RPM", 2000));

        Pose2d turretPose = shooter.getTurretPoseFromRobotPose(swerve.getPose());
        Strategy.StrategyConfig strategyConfig = Strategy.getLocationTarget(swerve.getPose().getTranslation());
        Translation2d locationTarget = strategyConfig.targetLocation;
        Distance distanceToLocationTarget = Meters.of(locationTarget.getDistance(turretPose.getTranslation()));

        SmartDashboard.putNumber("Distance For Testing", distanceToLocationTarget.in(Feet));

        shooter.setHoodAngle(shooter.targetHoodAngle(distanceToLocationTarget.in(Feet)));
        shooter.setShooterSpeed(angularVelocity);

        if (shooter.shooterAtSpeed(100)) {
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