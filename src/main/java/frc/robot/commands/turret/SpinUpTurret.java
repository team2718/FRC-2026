package frc.robot.commands.turret;

import static edu.wpi.first.units.Units.RPM;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.TurretSubsystem;

public class SpinUpTurret extends Command {
    private final TurretSubsystem shooter;
    private final double targetRPM;

    public SpinUpTurret(TurretSubsystem shooter, double targetRPM) {
        this.shooter = shooter;
        this.targetRPM = targetRPM;
        addRequirements(shooter);
    }

    @Override
    public void initialize() {
        // shooter.setShooterSpeed(RPM.of(targetRPM));
    }

    @Override
    public boolean isFinished() {
        return true;
    }
}
