package frc.robot.commands.turret;

import edu.wpi.first.math.geometry.Rotation2d;
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
        turret.setAzimuthAngle(Rotation2d.fromDegrees(angle));
    }

    @Override
    public boolean isFinished() {
        return true;
    }
}
