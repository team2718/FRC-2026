package frc.robot.commands.turret;

import static edu.wpi.first.units.Units.Degrees;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.TurretSubsystem;

public class AimAndPrespinTurret extends Command {
    TurretSubsystem turret;
    double angle;

    public AimAndPrespinTurret(TurretSubsystem turret, double angle) {
        this.turret = turret;
        this.angle = angle;
        addRequirements(turret);
    }

    @Override
    public void initialize() {
        turret.setShooterSpeedRPM(3000);
        turret.setAzimuthAngle(Degrees.of(angle));
    }

    @Override
    public boolean isFinished() {
        return true;
    }
}
